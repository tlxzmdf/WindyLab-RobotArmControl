#!/usr/bin/env python3
"""真机手拖阶段：仅录制 /master/joint_states 轨迹。"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]


class MasterDragRecorder(Node):
    def __init__(self, out_csv: Path) -> None:
        super().__init__('master_drag_recorder')
        self.out_csv = out_csv
        self.rows: list[dict] = []
        self.t0: Optional[float] = None
        self.create_subscription(JointState, '/master/joint_states', self._cb, 50)
        self.get_logger().info(f'等待 /master/joint_states，输出: {out_csv}')

    def _cb(self, msg: JointState) -> None:
        q = {}
        for name in ARM_JOINTS:
            if name not in msg.name:
                return
            idx = msg.name.index(name)
            q[name] = float(msg.position[idx]) if idx < len(msg.position) else 0.0
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        row = {'t_sec': round(stamp - self.t0, 6)}
        for name in ARM_JOINTS:
            row[f'{name}_pos'] = q[name]
        self.rows.append(row)


def save(out_dir: Path, rows: list[dict]) -> float:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'master_joints.csv'
    if not rows:
        csv_path.write_text('', encoding='utf-8')
        return 0.0
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    duration = float(rows[-1]['t_sec'])
    meta = {
        'duration_sec': duration,
        'samples': len(rows),
        'recorded_at': datetime.now(timezone.utc).isoformat(),
        'topic': '/master/joint_states',
    }
    (out_dir / 'master_meta.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return duration


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = MasterDragRecorder(args.out)
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        duration = save(args.out, node.rows)
        node.destroy_node()
        rclpy.try_shutdown()

    if len(node.rows) < 20:
        print('[WARN] 主臂样本过少，请确认 master_arm_node 已启动', file=sys.stderr)
        return 2
    print(f'主臂轨迹已保存: {args.out}/master_joints.csv  duration={duration:.2f}s  n={len(node.rows)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
