#!/usr/bin/env python3
"""录制 Demo 运行数据，用于事后分析跟踪误差、频率与关节曲线。

订阅:
  - /joint_states          实际关节反馈
  - /student/joint_command 学生 Demo 指令

输出目录结构:
  <out_dir>/
    run_meta.json
    joint_states.csv
    joint_command.csv
    tracking_error.csv   # 按时间对齐后的 cmd - actual
    summary.txt

用法:
  source ~/arm/.pc_arm_env.sh
  python3 record_demo_run.py --duration 15 --out ~/arm/run_data/test1
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [f'joint{i}' for i in range(1, 8)]


def _stamp_sec(msg: JointState) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _extract(msg: JointState) -> tuple[float, list[float], list[float]]:
    pos = [0.0] * len(JOINT_NAMES)
    vel = [0.0] * len(JOINT_NAMES)
    for i, name in enumerate(JOINT_NAMES):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if idx < len(msg.position):
            pos[i] = float(msg.position[idx])
        if idx < len(msg.velocity):
            vel[i] = float(msg.velocity[idx])
    return _stamp_sec(msg), pos, vel


@dataclass
class StreamBuffer:
    label: str
    rows: list[dict] = field(default_factory=list)
    t0: Optional[float] = None

    def add(self, stamp: float, pos: list[float], vel: list[float]) -> None:
        if self.t0 is None:
            self.t0 = stamp
        t_rel = stamp - self.t0
        row = {'t_sec': round(t_rel, 6)}
        for i, name in enumerate(JOINT_NAMES):
            row[f'{name}_pos'] = pos[i]
            row[f'{name}_vel'] = vel[i]
        self.rows.append(row)


class DemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__('demo_run_recorder')
        self.states = StreamBuffer('joint_states')
        self.commands = StreamBuffer('joint_command')
        self.create_subscription(JointState, '/joint_states', self._on_state, 50)
        self.create_subscription(JointState, '/student/joint_command', self._on_cmd, 50)
        self.get_logger().info('录制中: /joint_states, /student/joint_command')

    def _on_state(self, msg: JointState) -> None:
        self.states.add(*_extract(msg))

    def _on_cmd(self, msg: JointState) -> None:
        self.commands.add(*_extract(msg))


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _nearest_row(rows: list[dict], t: float) -> Optional[dict]:
    if not rows:
        return None
    best = min(rows, key=lambda r: abs(r['t_sec'] - t))
    return best


def _build_tracking_error(states: list[dict], commands: list[dict]) -> list[dict]:
    out: list[dict] = []
    if not states or not commands:
        return out
    for s in states:
        c = _nearest_row(commands, s['t_sec'])
        if c is None:
            continue
        row = {'t_sec': s['t_sec']}
        for name in JOINT_NAMES:
            row[f'{name}_err'] = c[f'{name}_pos'] - s[f'{name}_pos']
        out.append(row)
    return out


def _estimate_hz(rows: list[dict]) -> float:
    if len(rows) < 2:
        return 0.0
    dt = rows[-1]['t_sec'] - rows[0]['t_sec']
    if dt <= 0:
        return 0.0
    return (len(rows) - 1) / dt


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return (sum(v * v for v in values) / len(values)) ** 0.5


def _write_summary(
    path: Path,
    meta: dict,
    states: list[dict],
    commands: list[dict],
    tracking: list[dict],
) -> None:
    lines = [
        f"Demo: {meta['demo']}",
        f"Arm type: {meta['arm_type']}",
        f"Duration requested: {meta['duration_sec']} s",
        f"Recorded at: {meta['recorded_at']}",
        f"Output: {meta['output_dir']}",
        '',
        f"joint_states samples: {len(states)}  (~{_estimate_hz(states):.1f} Hz)",
        f"joint_command samples: {len(commands)}  (~{_estimate_hz(commands):.1f} Hz)",
        f"tracking_error rows: {len(tracking)}",
        '',
        'Per-joint position RMS tracking error (rad):',
    ]
    for name in JOINT_NAMES:
        errs = [r[f'{name}_err'] for r in tracking]
        lines.append(f'  {name}: {_rms(errs):.6f}')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def save_run(
    out_dir: Path,
    demo: str,
    arm_type: str,
    duration_sec: float,
    states: StreamBuffer,
    commands: StreamBuffer,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    tracking = _build_tracking_error(states.rows, commands.rows)
    meta = {
        'demo': demo,
        'arm_type': arm_type,
        'duration_sec': duration_sec,
        'recorded_at': datetime.now(timezone.utc).isoformat(),
        'output_dir': str(out_dir),
        'joint_states_samples': len(states.rows),
        'joint_command_samples': len(commands.rows),
        'joint_states_hz': round(_estimate_hz(states.rows), 2),
        'joint_command_hz': round(_estimate_hz(commands.rows), 2),
        'topics': ['/joint_states', '/student/joint_command'],
    }
    _write_csv(out_dir / 'joint_states.csv', states.rows)
    _write_csv(out_dir / 'joint_command.csv', commands.rows)
    _write_csv(out_dir / 'tracking_error.csv', tracking)
    (out_dir / 'run_meta.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    _write_summary(out_dir / 'summary.txt', meta, states.rows, commands.rows, tracking)


def wait_for_topic(timeout_sec: float = 30.0) -> bool:
    import subprocess

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        proc = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if '/joint_states' in proc.stdout:
            return True
        time.sleep(0.5)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description='录制机械臂 Demo 运行数据')
    parser.add_argument('--duration', type=float, default=15.0, help='录制时长 (秒)')
    parser.add_argument('--demo', default='unknown', help='Demo 名称（写入元数据）')
    parser.add_argument('--arm-type', default='sim', help='sim 或 a_l1')
    parser.add_argument('--out', type=Path, required=True, help='输出目录')
    parser.add_argument('--wait-topic', type=float, default=30.0, help='等待 /joint_states 超时')
    args = parser.parse_args()

    if not wait_for_topic(args.wait_topic):
        print('[FAIL] 超时：未检测到 /joint_states，请先启动 student_arm.launch.py', file=sys.stderr)
        return 1

    rclpy.init()
    node = DemoRecorder()
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        save_run(args.out, args.demo, args.arm_type, args.duration, node.states, node.commands)
        node.get_logger().info(
            f'已保存 {len(node.states.rows)} 条 joint_states, '
            f'{len(node.commands.rows)} 条 joint_command -> {args.out}'
        )
        node.destroy_node()
        rclpy.try_shutdown()

    if len(node.states.rows) < 10:
        print('[WARN] joint_states 样本过少，请确认 launch 与 Demo 是否正常运行', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
