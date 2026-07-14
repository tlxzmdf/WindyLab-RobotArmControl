#!/usr/bin/env python3
"""Run 7-second tests for stabilization modes A/B/C and generate comparison plots + metrics table."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib
matplotlib.use('Agg')
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

_CJK_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(_CJK_FONT).exists():
    fm.fontManager.addfont(_CJK_FONT)
    plt.rcParams['font.family'] = fm.FontProperties(fname=_CJK_FONT).get_name()
plt.rcParams['axes.unicode_minus'] = False
import pinocchio as pin
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
WORKSPACE = ARM_ROOT / 'windylab_ws'
REPORT_DIR = PROJECT_ROOT / 'reports' / 'three_mode_test'
DURATION_SEC = 7.0
WARMUP_SEC = 0.5
SAMPLE_HZ = 100.0
MOUNT_BASE_OFFSET_Z = 0.02

MODES = {
    'A': {
        'label': '模式 A: IK + 纯运动学稳定',
        'use_ik_joint_control': 'true',
        'kinematic_stabilization': 'true',
    },
    'B': {
        'label': '模式 B: IK + 关节空间计算力矩控制',
        'use_ik_joint_control': 'true',
        'kinematic_stabilization': 'false',
    },
    'C': {
        'label': '模式 C: 任务空间操作空间控制',
        'use_ik_joint_control': 'false',
        'kinematic_stabilization': 'false',
    },
}

MOUNT_JOINTS = ['mount_tx', 'mount_ty', 'mount_tz', 'mount_rx', 'mount_ry', 'mount_rz']
ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]


def mount_to_se3(pos: np.ndarray, rpy: np.ndarray, z_offset: float = MOUNT_BASE_OFFSET_Z) -> pin.SE3:
    R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
    T_drone = pin.SE3(R, pos)
    T_offset = pin.SE3(np.eye(3), np.array([0.0, 0.0, z_offset]))
    return T_drone * T_offset


@dataclass
class Sample:
    t: float
    pos: np.ndarray
    rpy: np.ndarray
    pos_err_mm: float
    orient_err_deg: float


@dataclass
class ModeResult:
    mode: str
    label: str
    target_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    target_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    samples: list[Sample] = field(default_factory=list)

    @property
    def times(self) -> np.ndarray:
        return np.array([s.t for s in self.samples])

    @property
    def actual_pos(self) -> np.ndarray:
        return np.array([s.pos for s in self.samples])

    @property
    def actual_rpy(self) -> np.ndarray:
        return np.array([s.rpy for s in self.samples])

    def pos_errors_mm(self) -> np.ndarray:
        return np.array([s.pos_err_mm for s in self.samples])

    def orient_errors_deg(self) -> np.ndarray:
        return np.array([s.orient_err_deg for s in self.samples])


class JointStateRecorder(Node):
    def __init__(self, urdf_path: Path, warmup_sec: float, duration_sec: float):
        super().__init__('three_mode_test_recorder')
        self.warmup_sec = warmup_sec
        self.duration_sec = duration_sec
        self.t0 = time.time()
        self.recording = False
        self.done = False
        self.record_t0 = 0.0
        self.target_pos: Optional[np.ndarray] = None
        self.target_rpy: Optional[np.ndarray] = None
        self.target_rot: Optional[np.ndarray] = None
        self.samples: list[Sample] = []
        self._latest_js: Optional[JointState] = None

        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.ee_frame = self.model.getFrameId('link6')

        self.create_subscription(JointState, '/joint_states', self._store_joint, 100)
        self.create_timer(1.0 / SAMPLE_HZ, self._sample_timer)

    def _elapsed(self) -> float:
        return time.time() - self.t0

    def _store_joint(self, msg: JointState) -> None:
        self._latest_js = msg

    def _compute_pose(self, msg: JointState) -> Optional[tuple[np.ndarray, np.ndarray, pin.SE3]]:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        mount_vals, arm_q = [], []
        for jn in MOUNT_JOINTS:
            idx = name_to_idx.get(jn)
            if idx is None or idx >= len(msg.position):
                return None
            mount_vals.append(msg.position[idx])
        for jn in ARM_JOINTS:
            idx = name_to_idx.get(jn)
            if idx is None or idx >= len(msg.position):
                return None
            arm_q.append(msg.position[idx])

        mount_pos = np.array(mount_vals[:3])
        mount_rpy = np.array(mount_vals[3:6])
        q = np.array(arm_q)
        T_base_world = mount_to_se3(mount_pos, mount_rpy)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        T_ee_world = T_base_world * self.data.oMf[self.ee_frame]
        pos = T_ee_world.translation.copy()
        rpy = np.array(pin.rpy.matrixToRpy(T_ee_world.rotation))
        return pos, rpy, T_ee_world

    def _sample_timer(self) -> None:
        elapsed = self._elapsed()
        if elapsed < self.warmup_sec:
            return
        if not self.recording:
            self.recording = True
            self.record_t0 = time.time()
        record_elapsed = time.time() - self.record_t0
        if record_elapsed >= self.duration_sec:
            self.done = True
            return
        if self._latest_js is None:
            return

        result = self._compute_pose(self._latest_js)
        if result is None:
            return
        pos, rpy, T_ee_world = result

        if self.target_pos is None:
            self.target_pos = pos.copy()
            self.target_rpy = rpy.copy()
            self.target_rot = T_ee_world.rotation.copy()

        pos_err_mm = float(np.linalg.norm(pos - self.target_pos)) * 1000.0
        orient_err_deg = float(np.degrees(
            np.linalg.norm(pin.log3(T_ee_world.rotation.T @ self.target_rot))))

        self.samples.append(Sample(
            t=record_elapsed,
            pos=pos,
            rpy=rpy,
            pos_err_mm=pos_err_mm,
            orient_err_deg=orient_err_deg,
        ))


def kill_stabilization_procs() -> None:
    for pat in (
        'stabilization_headless.launch',
        'arm_ee_stabilization_control/ee_stabilization',
        'arm_ee_stabilization_description/urdf/arm_on_drone',
    ):
        subprocess.run(['pkill', '-TERM', '-f', pat], stderr=subprocess.DEVNULL)
    time.sleep(1.5)


def start_stack(mode_cfg: dict) -> subprocess.Popen:
    env = os.environ.copy()
    ros_setup = '/opt/ros/humble/setup.bash'
    ws_setup = str(WORKSPACE / 'install/setup.bash')
    cmd = (
        f'source {ros_setup} && source {ws_setup} && '
        f'ros2 launch arm_ee_stabilization_description stabilization_headless.launch.py '
        f'use_ik_joint_control:={mode_cfg["use_ik_joint_control"]} '
        f'kinematic_stabilization:={mode_cfg["kinematic_stabilization"]}'
    )
    proc = subprocess.Popen(
        ['bash', '-lc', cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        env=env,
    )
    time.sleep(3.0)
    return proc


def stop_proc(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    time.sleep(0.5)
    kill_stabilization_procs()


def record_mode(mode_key: str, mode_cfg: dict, urdf_path: Path) -> ModeResult:
    kill_stabilization_procs()
    proc = start_stack(mode_cfg)

    rclpy.init()
    recorder = JointStateRecorder(urdf_path, WARMUP_SEC, DURATION_SEC)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(recorder)
    try:
        deadline = time.time() + WARMUP_SEC + DURATION_SEC + 3.0
        while rclpy.ok() and not recorder.done and time.time() < deadline:
            if proc.poll() is not None:
                out = proc.stdout.read().decode(errors='replace') if proc.stdout else ''
                print(f'  WARNING: launch exited early (code={proc.returncode})')
                if out:
                    print(out[-800:])
                break
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(recorder)
        recorder.destroy_node()
        rclpy.try_shutdown()
    stop_proc(proc)

    result = ModeResult(mode=mode_key, label=mode_cfg['label'])
    if recorder.target_pos is not None:
        result.target_pos = recorder.target_pos
        result.target_rpy = recorder.target_rpy
    result.samples = recorder.samples
    return result


def plot_mode_result(result: ModeResult, out_dir: Path) -> None:
    if len(result.samples) < 10:
        print(f'WARNING: mode {result.mode} has only {len(result.samples)} samples')
        return

    times = result.times
    actual_pos = result.actual_pos
    actual_rpy = np.degrees(result.actual_rpy)
    target_pos = result.target_pos
    target_rpy = np.degrees(result.target_rpy)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f'{result.label} — 7 秒位置与姿态跟踪', fontsize=14, fontweight='bold')

    blues = ['#0D47A1', '#1565C0', '#42A5F5']
    pos_names = ['X', 'Y', 'Z']
    rpy_names = ['Roll', 'Pitch', 'Yaw']

    for i in range(3):
        axes[0].plot(times, np.full_like(times, target_pos[i]), 'r-', lw=2.0,
                     label=f'目标 {pos_names[i]}', alpha=0.85)
        axes[0].plot(times, actual_pos[:, i], color=blues[i], lw=1.4,
                     label=f'实际 {pos_names[i]}', alpha=0.95)
    axes[0].set_ylabel('位置 (m)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right', fontsize=9)
    axes[0].set_title('世界系位置 — 红色=目标期望值, 蓝色=实际效果')

    for i in range(3):
        axes[1].plot(times, np.full_like(times, target_rpy[i]), 'r-', lw=2.0, alpha=0.85,
                     label=f'目标 {rpy_names[i]}')
        axes[1].plot(times, actual_rpy[:, i], color=blues[i], lw=1.4, alpha=0.95,
                     label=f'实际 {rpy_names[i]}')
    axes[1].legend(loc='upper right', fontsize=8, ncol=2)
    axes[0].set_xlim(0, DURATION_SEC)
    axes[1].set_xlim(0, DURATION_SEC)
    axes[1].set_ylabel('姿态角 (°)')
    axes[1].set_xlabel('时间 (s)')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('世界系姿态 Roll/Pitch/Yaw — 红色=目标期望值, 蓝色=实际效果')

    pos_err = result.pos_errors_mm()
    ori_err = result.orient_errors_deg()
    stats_text = (
        f'位置误差: 平均={pos_err.mean():.3f} mm, 最大={pos_err.max():.3f} mm, RMS={np.sqrt(np.mean(pos_err**2)):.3f} mm\n'
        f'姿态误差: 平均={ori_err.mean():.4f}°, 最大={ori_err.max():.4f}°, RMS={np.sqrt(np.mean(ori_err**2)):.4f}°'
    )
    fig.text(0.5, 0.01, stats_text, ha='center', fontsize=10,
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout(rect=[0, 0.04, 1, 0.96])
    out_path = out_dir / f'mode_{result.mode}_position_orientation.png'
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved plot: {out_path}')


def compute_metrics(result: ModeResult) -> dict:
    pos_err = result.pos_errors_mm()
    ori_err = result.orient_errors_deg()
    if pos_err.size == 0:
        return {'mode': result.mode, 'label': result.label, 'samples': 0}
    return {
        'mode': result.mode,
        'label': result.label,
        'samples': int(len(result.samples)),
        'pos_mean_mm': float(pos_err.mean()),
        'pos_max_mm': float(pos_err.max()),
        'pos_rms_mm': float(np.sqrt(np.mean(pos_err ** 2))),
        'pos_p95_mm': float(np.percentile(pos_err, 95)),
        'orient_mean_deg': float(ori_err.mean()),
        'orient_max_deg': float(ori_err.max()),
        'orient_rms_deg': float(np.sqrt(np.mean(ori_err ** 2))),
        'orient_p95_deg': float(np.percentile(ori_err, 95)),
        'target_pos_m': result.target_pos.tolist(),
        'target_rpy_deg': np.degrees(result.target_rpy).tolist(),
    }


def write_report(metrics: list[dict], out_dir: Path) -> None:
    csv_path = out_dir / 'error_summary.csv'
    md_path = out_dir / 'test_report.md'

    fieldnames = [
        'mode', 'label', 'samples',
        'pos_mean_mm', 'pos_max_mm', 'pos_rms_mm', 'pos_p95_mm',
        'orient_mean_deg', 'orient_max_deg', 'orient_rms_deg', 'orient_p95_deg',
    ]
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for m in metrics:
            writer.writerow(m)
    print(f'Saved CSV: {csv_path}')

    with open(out_dir / 'metrics.json', 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    lines = [
        '# 机头稳定三模式对比测试报告',
        '',
        f'- 测试时长: {DURATION_SEC:.0f} 秒/模式 (预热 {WARMUP_SEC:.1f} 秒)',
        f'- 采样频率: ~{SAMPLE_HZ:.0f} Hz (joint_states 驱动)',
        f'- 扰动: 球半径 0.35 m, 姿态幅值 0.32 rad (固定 seed=42)',
        '',
        '## 误差统计表',
        '',
        '| 模式 | 说明 | 样本数 | 位置平均误差 (mm) | 位置最大误差 (mm) | 位置 RMS (mm) | 位置 P95 (mm) | '
        '姿态平均误差 (°) | 姿态最大误差 (°) | 姿态 RMS (°) | 姿态 P95 (°) |',
        '|------|------|--------|-------------------|-------------------|---------------|---------------|'
        '------------------|------------------|--------------|--------------|',
    ]
    mode_desc = {
        'A': 'IK + 纯运动学稳定',
        'B': 'IK + 关节空间 CT',
        'C': '任务空间 OSC',
    }
    for m in metrics:
        if m.get('samples', 0) == 0:
            lines.append(f"| {m['mode']} | {mode_desc.get(m['mode'], '')} | 0 | — | — | — | — | — | — | — | — |")
            continue
        lines.append(
            f"| **{m['mode']}** | {mode_desc.get(m['mode'], '')} | {m['samples']} "
            f"| {m['pos_mean_mm']:.3f} | {m['pos_max_mm']:.3f} | {m['pos_rms_mm']:.3f} | {m['pos_p95_mm']:.3f} "
            f"| {m['orient_mean_deg']:.4f} | {m['orient_max_deg']:.4f} | {m['orient_rms_deg']:.4f} | {m['orient_p95_deg']:.4f} |"
        )
    lines += [
        '',
        '## 图表 (7 秒位置/姿态时序)',
        '',
        '图例: **红色** = 目标期望值 (锁定世界系位姿), **蓝色** = 实际末端位姿',
        '',
        '| 模式 | 图表 |',
        '|------|------|',
    ]
    for key in MODES:
        lines.append(f'| {key} | ![mode_{key}](mode_{key}_position_orientation.png) |')
    md_path.write_text('\n'.join(lines), encoding='utf-8')
    print(f'Saved report: {md_path}')


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Three-mode EE stabilization test')
    p.add_argument('--duration', type=float, default=DURATION_SEC)
    p.add_argument('--warmup', type=float, default=WARMUP_SEC)
    p.add_argument('--output', type=str, default=str(REPORT_DIR))
    p.add_argument('--modes', type=str, default='A,B,C')
    return p.parse_args()


def main() -> None:
    args = parse_args()
    global DURATION_SEC, WARMUP_SEC
    DURATION_SEC = args.duration
    WARMUP_SEC = args.warmup
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    urdf_path = (
        WORKSPACE / 'install/arm_ee_stabilization_description/share'
        '/arm_ee_stabilization_description/urdf/single_arm.urdf'
    )
    if not urdf_path.exists():
        print(f'ERROR: URDF not found: {urdf_path}', file=sys.stderr)
        sys.exit(1)

    mode_keys = [m.strip().upper() for m in args.modes.split(',') if m.strip()]
    results: list[ModeResult] = []
    all_metrics: list[dict] = []

    for key in mode_keys:
        if key not in MODES:
            print(f'Skip unknown mode {key}')
            continue
        print(f'\n{"="*60}\nRunning {MODES[key]["label"]}\n{"="*60}')
        result = record_mode(key, MODES[key], urdf_path)
        print(f'  Recorded {len(result.samples)} samples')
        if len(result.samples) >= 10:
            plot_mode_result(result, out_dir)
        else:
            print(f'  WARNING: insufficient samples for mode {key}')
        results.append(result)
        all_metrics.append(compute_metrics(result))

    write_report(all_metrics, out_dir)

    print('\n' + '=' * 70)
    print('误差统计汇总')
    print('=' * 70)
    hdr = f"{'模式':<6} {'样本':>6} {'位置mean(mm)':>14} {'位置max(mm)':>14} {'姿态mean(°)':>14} {'姿态max(°)':>14}"
    print(hdr)
    print('-' * 70)
    for m in all_metrics:
        if m.get('samples', 0) == 0:
            print(f"{m['mode']:<6} {'0':>6} {'N/A':>14} {'N/A':>14} {'N/A':>14} {'N/A':>14}")
        else:
            print(
                f"{m['mode']:<6} {m['samples']:>6} {m['pos_mean_mm']:>14.3f} {m['pos_max_mm']:>14.3f} "
                f"{m['orient_mean_deg']:>14.4f} {m['orient_max_deg']:>14.4f}"
            )
    print(f'\nReports saved to: {out_dir}')


if __name__ == '__main__':
    main()
