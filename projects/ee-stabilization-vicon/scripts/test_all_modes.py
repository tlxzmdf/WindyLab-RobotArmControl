#!/usr/bin/env python3
"""三模式机头稳定测试: 规划(末端位姿) + 控制(关节) 7s 曲线与误差统计."""

from __future__ import annotations

import argparse
import csv
import json
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
import pinocchio as pin
import rclpy
from scipy.signal import welch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
import tf2_ros

_CJK_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(_CJK_FONT).exists():
    fm.fontManager.addfont(_CJK_FONT)
    plt.rcParams['font.family'] = fm.FontProperties(fname=_CJK_FONT).get_name()
plt.rcParams['axes.unicode_minus'] = False

# 文档插图字号（图 2–7 等）
plt.rcParams.update({
    'font.size': 13,
    'axes.titlesize': 14,
    'axes.labelsize': 13,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
})
FIG_DPI = 180

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
WS = ARM_ROOT / 'windylab_ws'
REPORT_DIR = PROJECT_ROOT / 'reports'
DOCS_DIR = PROJECT_ROOT / 'docs'
WARMUP_SEC = 2.5
RECORD_SEC = 7.0
EE_FRAME = 'link6'
ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]

MODES = {
    'A': {
        'label': '模式 A: IK + 纯运动学稳定',
        'planning_label': '规划 A: IK 关节映射 + 运动学执行',
        'control_label': '控制 A: 理想运动学赋值',
        'use_ik_joint_control': True,
        'kinematic_stabilization': True,
    },
    'B': {
        'label': '模式 B: IK + 关节空间计算力矩控制',
        'planning_label': '规划 B: IK 关节映射 + 低通滤波',
        'control_label': '控制 B: 关节空间计算力矩 (CTC)',
        'use_ik_joint_control': True,
        'kinematic_stabilization': False,
    },
    'C': {
        'label': '模式 C: 任务空间操作空间控制',
        'planning_label': '规划 C: 任务空间期望位姿 (无关节映射)',
        'control_label': '控制 C: 操作空间力矩 (OSC)',
        'use_ik_joint_control': False,
        'kinematic_stabilization': False,
    },
    'D': {
        'label': '模式 D: sat 速度规划 + OSC + ESO',
        'planning_label': '规划 D: Wang 式 sat 关节速度参考',
        'control_label': '控制 D: OSC + 轻量 ESO 扰动补偿',
        'stabilization_mode': 'D',
        'use_ik_joint_control': False,
        'kinematic_stabilization': False,
    },
}


def quat_to_rpy(w: float, x: float, y: float, z: float) -> np.ndarray:
    return pin.rpy.matrixToRpy(pin.Quaternion(w, x, y, z).matrix())


@dataclass
class Sample:
    t: float
    pos: np.ndarray
    rpy: np.ndarray
    q_act: np.ndarray
    q_cmd: np.ndarray
    q_plan: np.ndarray


@dataclass
class ModeRecord:
    mode_id: str
    label: str
    samples: list[Sample] = field(default_factory=list)
    target_pos: Optional[np.ndarray] = None
    target_rpy: Optional[np.ndarray] = None


def _stamp_nsec(msg: JointState) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _extract_arm_q(msg: JointState) -> Optional[np.ndarray]:
    q = np.zeros(6)
    got = 0
    for i, name in enumerate(ARM_JOINTS):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if idx < len(msg.position):
            q[i] = msg.position[idx]
            got += 1
    return q if got == len(ARM_JOINTS) else None


def _extract_ref(msg: JointState) -> Optional[tuple[np.ndarray, np.ndarray]]:
    q_plan = np.zeros(6)
    q_cmd = np.zeros(6)
    got = 0
    for i, name in enumerate(ARM_JOINTS):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if idx < len(msg.position):
            q_plan[i] = msg.position[idx]
        if idx < len(msg.velocity):
            q_cmd[i] = msg.velocity[idx]
        elif idx < len(msg.position):
            q_cmd[i] = msg.position[idx]
        got += 1
    return (q_plan, q_cmd) if got == len(ARM_JOINTS) else None


class ModeRecorder(Node):
    def __init__(self, warmup_sec: float):
        super().__init__('mode_test_recorder')
        self.warmup_sec = warmup_sec
        self.t0 = time.time()
        self.recording = False
        self.t_record0_nsec: Optional[int] = None
        self._last_sample_t = -1.0
        self.target_pos: Optional[np.ndarray] = None
        self.target_rpy: Optional[np.ndarray] = None
        self.samples: list[Sample] = []
        self._js_by_stamp: dict[int, np.ndarray] = {}
        self._ref_by_stamp: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(MarkerArray, '/stabilization_markers', self._marker_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 50)
        self.create_subscription(JointState, '/stabilization_reference', self._ref_cb, 50)

    def _marker_cb(self, msg: MarkerArray) -> None:
        if self.target_pos is not None:
            return
        for marker in msg.markers:
            if marker.id == 1:
                p = marker.pose.position
                o = marker.pose.orientation
                self.target_pos = np.array([p.x, p.y, p.z])
                self.target_rpy = quat_to_rpy(o.w, o.x, o.y, o.z)
                break

    def _joint_cb(self, msg: JointState) -> None:
        q_act = _extract_arm_q(msg)
        if q_act is None:
            return
        stamp = _stamp_nsec(msg)
        self._js_by_stamp[stamp] = q_act
        if len(self._js_by_stamp) > 2000:
            for key in sorted(self._js_by_stamp)[:-1000]:
                self._js_by_stamp.pop(key, None)
        self._try_pair(stamp)

    def _ref_cb(self, msg: JointState) -> None:
        ref = _extract_ref(msg)
        if ref is None:
            return
        stamp = _stamp_nsec(msg)
        self._ref_by_stamp[stamp] = ref
        if len(self._ref_by_stamp) > 2000:
            for key in sorted(self._ref_by_stamp)[:-1000]:
                self._ref_by_stamp.pop(key, None)
        self._try_pair(stamp)

    def _elapsed(self) -> float:
        return time.time() - self.t0

    def _try_pair(self, stamp: int) -> None:
        if stamp not in self._js_by_stamp or stamp not in self._ref_by_stamp:
            return
        if self._elapsed() < self.warmup_sec:
            return
        if self.t_record0_nsec is None:
            self.t_record0_nsec = stamp
            self.recording = True
        t = (stamp - self.t_record0_nsec) / 1e9
        if t >= RECORD_SEC:
            return
        if t - self._last_sample_t < 0.018:
            return
        q_act = self._js_by_stamp[stamp]
        q_plan, q_cmd = self._ref_by_stamp[stamp]
        try:
            tf_time = rclpy.time.Time(seconds=stamp // 1_000_000_000,
                                      nanoseconds=stamp % 1_000_000_000)
            tf = self._tf_buffer.lookup_transform('world', EE_FRAME, tf_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            try:
                tf = self._tf_buffer.lookup_transform('world', EE_FRAME, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return
        tr = tf.transform.translation
        qr = tf.transform.rotation
        pos = np.array([tr.x, tr.y, tr.z])
        rpy = quat_to_rpy(qr.w, qr.x, qr.y, qr.z)
        self._last_sample_t = t
        self.samples.append(Sample(
            t=t,
            pos=pos.copy(),
            rpy=rpy.copy(),
            q_act=q_act.copy(),
            q_cmd=q_cmd.copy(),
            q_plan=q_plan.copy(),
        ))


def kill_stale() -> None:
    patterns = [
        'ee_stabilization',
        'stabilization.launch',
        'stabilization_headless.launch',
        'robot_state_publisher.*arm_on_drone',
        'rviz2',
    ]
    for pat in patterns:
        subprocess.run(['pkill', '-f', pat], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def launch_mode(mode_cfg: dict) -> subprocess.Popen:
    env = os.environ.copy()
    humble = '/opt/ros/humble/setup.bash'
    ws_setup = str(WS / 'install' / 'setup.bash')
    ik = 'true' if mode_cfg['use_ik_joint_control'] else 'false'
    kin = 'true' if mode_cfg['kinematic_stabilization'] else 'false'
    mode = mode_cfg.get('stabilization_mode', '')
    mode_arg = f' stabilization_mode:={mode}' if mode else ''
    bash_cmd = f'''
source {humble}
source {ws_setup}
ros2 launch arm_ee_stabilization_description stabilization_headless.launch.py \
  use_ik_joint_control:={ik} kinematic_stabilization:={kin}{mode_arg}
'''
    return subprocess.Popen(
        ['bash', '-lc', bash_cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=env,
        preexec_fn=os.setsid,
    )


def record_mode(mode_id: str, label: str) -> ModeRecord:
    kill_stale()
    proc = launch_mode(MODES[mode_id])
    time.sleep(5.0)
    rclpy.init()
    node = ModeRecorder(warmup_sec=WARMUP_SEC)
    deadline = time.time() + WARMUP_SEC + RECORD_SEC + 5.0
    try:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        record = ModeRecord(
            mode_id=mode_id,
            label=label,
            samples=node.samples,
            target_pos=node.target_pos,
            target_rpy=node.target_rpy,
        )
        node.destroy_node()
        rclpy.try_shutdown()
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    if len(record.samples) < 50:
        raise RuntimeError(
            f'{mode_id}: 样本不足 ({len(record.samples)}), '
            f'stderr={proc.stderr.read().decode()[:500] if proc.stderr else ""}')
    if record.target_pos is None:
        raise RuntimeError(f'{mode_id}: 未收到目标 Marker (id=1)')
    return record


def effective_q_cmd(samples: list[Sample]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return times, q_act, q_cmd arrays."""
    times = np.array([s.t for s in samples])
    q_act = np.array([s.q_act for s in samples])
    q_cmd = np.array([s.q_cmd for s in samples])
    return times, q_act, q_cmd


def compute_stats(record: ModeRecord) -> dict:
    times = np.array([s.t for s in record.samples])
    pos = np.array([s.pos for s in record.samples])
    rpy = np.array([s.rpy for s in record.samples])
    tgt_p = record.target_pos
    tgt_r = record.target_rpy
    assert tgt_p is not None and tgt_r is not None

    pos_err = np.linalg.norm(pos - tgt_p, axis=1)
    rpy_err = np.linalg.norm(rpy - tgt_r, axis=1)

    def stat(v: np.ndarray) -> dict:
        return {
            'mean': float(v.mean()),
            'max': float(v.max()),
            'rms': float(np.sqrt(np.mean(v * v))),
            'p95': float(np.percentile(v, 95)),
        }

    _, q_act, q_cmd = effective_q_cmd(record.samples)
    q_joint_err = np.linalg.norm(q_act - q_cmd, axis=1)

    return {
        'samples': len(record.samples),
        'position_norm_mm': stat(pos_err * 1000.0),
        'orientation_norm_deg': stat(np.degrees(rpy_err)),
        'joint_cmd_norm_rad': stat(q_joint_err),
    }


def plot_planning(record: ModeRecord, out_path: Path) -> None:
    """规划层: 世界系末端位置与姿态 — 期望 vs 实际."""
    times = np.array([s.t for s in record.samples])
    pos = np.array([s.pos for s in record.samples])
    rpy = np.degrees(np.array([s.rpy for s in record.samples]))
    tgt_p = record.target_pos
    tgt_r = np.degrees(record.target_rpy)
    assert tgt_p is not None and tgt_r is not None

    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    fig.suptitle(
        f'{MODES[record.mode_id]["planning_label"]} — 末端规划跟踪 ({RECORD_SEC:.0f}s)',
        fontsize=16,
    )
    pos_labels = ['X (m)', 'Y (m)', 'Z (m)']
    ang_labels = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
    for i, ax in enumerate(axes[0]):
        ax.plot(times, np.full_like(times, tgt_p[i]), 'r-', lw=1.5, label='期望')
        ax.plot(times, pos[:, i], 'b-', lw=1.2, alpha=0.9, label='实际')
        ax.set_title(pos_labels[i])
        ax.set_xlabel('时间 (s)')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc='upper right', fontsize=12)
    for i, ax in enumerate(axes[1]):
        ax.plot(times, np.full_like(times, tgt_r[i]), 'r-', lw=1.5, label='期望')
        ax.plot(times, rpy[:, i], 'b-', lw=1.2, alpha=0.9, label='实际')
        ax.set_title(ang_labels[i])
        ax.set_xlabel('时间 (s)')
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close()
    print(f'  规划图: {out_path}')


def plot_control(record: ModeRecord, out_path: Path) -> None:
    """控制层: 六关节 — 上排跟踪曲线(局部放大), 下排跟踪误差(mrad)."""
    times, q_act, q_cmd = effective_q_cmd(record.samples)
    q_err = q_act - q_cmd

    fig, axes = plt.subplots(4, 3, figsize=(15, 12), sharex='col')
    fig.suptitle(
        f'{MODES[record.mode_id]["control_label"]} — 关节控制 ({RECORD_SEC:.0f}s)',
        fontsize=16,
        y=0.995,
    )
    fig.text(
        0.5, 0.965,
        '上：期望(红虚线)/实际(蓝实线)  下：跟踪误差(mrad)  '
        '（模式 C 由 OSC 控制，关节误差含 IK 冗余跳变）',
        ha='center', fontsize=12, color='0.25',
    )

    for i in range(6):
        ax_tr = axes[i // 3, i % 3]
        ax_er = axes[2 + i // 3, i % 3]
        ref = q_cmd[:, i]
        act = q_act[:, i]
        err_mrad = q_err[:, i] * 1000.0

        ax_tr.plot(times, ref, 'r--', lw=1.6, label='期望', dashes=(6, 3))
        ax_tr.plot(times, act, 'b-', lw=1.4, alpha=0.95, label='实际')
        err_max = float(np.max(np.abs(q_err[:, i])))
        span = float(np.max(ref) - np.min(ref))
        pad = max(err_max * 8.0, span * 0.015, 1e-5)
        mid = 0.5 * (float(np.max(ref)) + float(np.min(ref)))
        if span > 2.0 * pad:
            ax_tr.set_ylim(float(np.min(ref)) - pad, float(np.max(ref)) + pad)
        else:
            ax_tr.set_ylim(mid - pad, mid + pad)
        ax_tr.set_title(f'{ARM_JOINTS[i]} · 期望/实际')
        ax_tr.set_ylabel('角度 (rad)')
        ax_tr.grid(True, alpha=0.3)
        if i == 0:
            ax_tr.legend(loc='upper right', fontsize=12)

        ax_er.set_title(f'{ARM_JOINTS[i]} · 误差')
        ax_er.plot(times, err_mrad, 'k-', lw=1.2)
        ax_er.axhline(0.0, color='0.6', lw=0.8, ls='--')
        ax_er.fill_between(times, err_mrad, 0.0, alpha=0.25, color='C0')
        ax_er.set_ylabel('误差 (mrad)')
        ax_er.set_xlabel('时间 (s)')
        ax_er.grid(True, alpha=0.3)
        rms_mrad = float(np.sqrt(np.mean(err_mrad ** 2)))
        ax_er.text(
            0.02, 0.95, f'RMS={rms_mrad:.3f} mrad',
            transform=ax_er.transAxes, fontsize=11, va='top',
        )

    plt.tight_layout(rect=(0, 0, 1, 0.94))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close()
    print(f'  控制图: {out_path}')


def orientation_error_series(record: ModeRecord) -> tuple[np.ndarray, np.ndarray]:
    """Return (times, ||rpy - target_rpy|| in rad)."""
    times = np.array([s.t for s in record.samples])
    rpy = np.array([s.rpy for s in record.samples])
    tgt_r = record.target_rpy
    assert tgt_r is not None
    err = np.linalg.norm(rpy - tgt_r, axis=1)
    return times, err


def save_timeseries_npz(records: dict[str, ModeRecord], path: Path) -> None:
    payload = {}
    for mode_id, rec in records.items():
        times = np.array([s.t for s in rec.samples])
        pos = np.array([s.pos for s in rec.samples])
        rpy = np.array([s.rpy for s in rec.samples])
        _, ori_err = orientation_error_series(rec)
        payload[f'{mode_id}_t'] = times
        payload[f'{mode_id}_pos'] = pos
        payload[f'{mode_id}_rpy'] = rpy
        payload[f'{mode_id}_ori_err_rad'] = ori_err
        payload[f'{mode_id}_q_act'] = np.array([s.q_act for s in rec.samples])
        payload[f'{mode_id}_q_cmd'] = np.array([s.q_cmd for s in rec.samples])
        payload[f'{mode_id}_target_pos'] = rec.target_pos
        payload[f'{mode_id}_target_rpy'] = rec.target_rpy
    np.savez_compressed(path, **payload)
    print(f'  时序数据: {path}')


def load_timeseries_npz(path: Path) -> dict[str, ModeRecord]:
    data = np.load(path, allow_pickle=True)
    records: dict[str, ModeRecord] = {}
    for mode_id in ('A', 'B', 'C', 'D'):
        t_key = f'{mode_id}_t'
        if t_key not in data:
            continue
        times = data[t_key]
        pos = data[f'{mode_id}_pos']
        rpy = data[f'{mode_id}_rpy']
        q_act = data[f'{mode_id}_q_act'] if f'{mode_id}_q_act' in data else np.zeros((len(times), 6))
        q_cmd = data[f'{mode_id}_q_cmd'] if f'{mode_id}_q_cmd' in data else np.zeros((len(times), 6))
        tgt_p = data[f'{mode_id}_target_pos']
        tgt_r = data[f'{mode_id}_target_rpy']
        samples = [
            Sample(t=float(times[i]), pos=pos[i], rpy=rpy[i],
                   q_act=q_act[i], q_cmd=q_cmd[i], q_plan=q_cmd[i])
            for i in range(len(times))
        ]
        records[mode_id] = ModeRecord(
            mode_id=mode_id,
            label=MODES[mode_id]['label'],
            samples=samples,
            target_pos=tgt_p,
            target_rpy=tgt_r,
        )
    return records


def plot_attitude_timeseries(records: dict[str, ModeRecord], out_path: Path) -> None:
    """姿态误差范数时域对比（三模式）。"""
    fig, ax = plt.subplots(figsize=(12, 4))
    colors = {'A': 'C2', 'B': 'C0', 'C': 'C1', 'D': 'C4'}
    for mode_id in ('A', 'B', 'C', 'D'):
        if mode_id not in records:
            continue
        times, err = orientation_error_series(records[mode_id])
        rms = float(np.sqrt(np.mean(np.degrees(err) ** 2)))
        ax.plot(times, np.degrees(err), lw=1.2, color=colors[mode_id],
                label=f'模式 {mode_id} (RMS={rms:.3f}°)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel(r'$\|e_R\|$ (°)')
    ax.set_title('姿态误差时域对比', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=12)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close()
    print(f'  姿态时域图: {out_path}')


def plot_attitude_psd(records: dict[str, ModeRecord], out_path: Path) -> None:
    """模式 B/C 姿态误差 Welch 功率谱对比。"""
    fig, ax = plt.subplots(figsize=(10, 4))
    for mode_id, color in (('B', 'C0'), ('C', 'C1')):
        if mode_id not in records:
            continue
        times, err = orientation_error_series(records[mode_id])
        if len(times) < 32:
            continue
        dt = float(np.median(np.diff(times)))
        fs = 1.0 / dt
        err_deg = np.degrees(err - err.mean())
        nperseg = min(128, len(err_deg) // 2)
        freqs, psd = welch(err_deg, fs=fs, nperseg=nperseg)
        ax.semilogy(freqs, psd, lw=1.5, color=color, label=f'模式 {mode_id}')
    ax.axvline(0.5, color='0.5', ls='--', lw=1.0, label='扰动主频 $\sim$0.5\,Hz')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel(r'PSD ($\mathrm{deg}^2/\mathrm{Hz}$)')
    ax.set_title('姿态误差功率谱（B vs C）', fontsize=14)
    ax.set_xlim(0.0, 5.0)
    ax.grid(True, alpha=0.3, which='both')
    ax.legend(loc='upper right')
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close()
    print(f'  姿态频域图: {out_path}')


def plot_attitude_rpy(records: dict[str, ModeRecord], out_path: Path) -> None:
    """模式 C Roll/Pitch/Yaw 角速度（数值差分）。"""
    if 'C' not in records:
        return
    rec = records['C']
    times = np.array([s.t for s in rec.samples])
    rpy_deg = np.degrees(np.array([s.rpy for s in rec.samples]))
    dt = np.diff(times)
    dt[dt < 1e-6] = np.median(dt)
    rpy_rate = np.diff(rpy_deg, axis=0) / dt[:, None]

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    labels = ['Roll rate (°/s)', 'Pitch rate (°/s)', 'Yaw rate (°/s)']
    for i, ax in enumerate(axes):
        ax.plot(times[1:], rpy_rate[:, i], 'b-', lw=1.0)
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('时间 (s)')
    fig.suptitle('模式 C：末端 RPY 角速度（数值差分）', fontsize=16)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close()
    print(f'  角速度图: {out_path}')


def copy_to_docs(report_path: Path, docs_name: str) -> None:
    import shutil
    dst = DOCS_DIR / docs_name
    shutil.copy2(report_path, dst)


def write_summary_table(all_stats: dict, out_md: Path, out_csv: Path) -> None:
    mode_ids = [m for m in ('A', 'B', 'C', 'D') if m in all_stats]
    rows = []
    for mode_id in mode_ids:
        s = all_stats[mode_id]
        rows.append({
            '模式': MODES[mode_id]['label'],
            '样本数': s['samples'],
            '位置平均误差(mm)': f"{s['position_norm_mm']['mean']:.3f}",
            '位置最大误差(mm)': f"{s['position_norm_mm']['max']:.3f}",
            '角度平均误差(°)': f"{s['orientation_norm_deg']['mean']:.4f}",
            '角度最大误差(°)': f"{s['orientation_norm_deg']['max']:.4f}",
            '关节指令误差(rad)': f"{s['joint_cmd_norm_rad']['mean']:.4f}",
        })
    out_md.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        '# 机头稳定三模式测试报告',
        '',
        f'- 记录时长: 预热 {WARMUP_SEC:.1f}s + 记录 {RECORD_SEC:.0f}s',
        f'- 末端帧: `world → {EE_FRAME}`',
        '',
        '## 规划层误差（末端位姿）',
        '',
        '| ' + ' | '.join(rows[0].keys()) + ' |',
        '| ' + ' | '.join(['---'] * len(rows[0])) + ' |',
    ]
    for row in rows:
        lines.append('| ' + ' | '.join(str(row[k]) for k in row) + ' |')
    lines += [
        '',
        '## 曲线图',
        '',
        '| 模式 | 规划 (末端) | 控制 (关节) |',
        '| --- | --- | --- |',
    ]
    for mode_id in mode_ids:
        lines.append(
            f"| {MODES[mode_id]['label']} | "
            f"`mode_{mode_id}_planning.png` | `mode_{mode_id}_control.png` |"
        )
    out_md.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'  报告: {out_md}')


def print_table(all_stats: dict) -> None:
    print('\n' + '=' * 80)
    print('三模式规划层误差统计（末端 world→link6）')
    print('=' * 80)
    print(f"{'模式':<36} {'位置均值(mm)':>12} {'位置最大(mm)':>12} "
          f"{'角度均值(°)':>12} {'关节误差(rad)':>14}")
    print('-' * 80)
    mode_ids = [m for m in ('A', 'B', 'C', 'D') if m in all_stats]
    for mode_id in mode_ids:
        s = all_stats[mode_id]
        print(
            f"{MODES[mode_id]['label']:<36} "
            f"{s['position_norm_mm']['mean']:>12.3f} "
            f"{s['position_norm_mm']['max']:>12.3f} "
            f"{s['orientation_norm_deg']['mean']:>12.4f} "
            f"{s['joint_cmd_norm_rad']['mean']:>14.4f}"
        )
    print('=' * 80)


def main() -> None:
    parser = argparse.ArgumentParser(description='机头稳定三模式规划+控制对比测试')
    parser.add_argument('--modes', nargs='+', default=['A', 'B', 'C'], choices=['A', 'B', 'C', 'D'])
    parser.add_argument('--report-dir', type=Path, default=REPORT_DIR)
    parser.add_argument('--skip-docs-copy', action='store_true')
    parser.add_argument('--plot-from-npz', type=Path, default=None,
                        help='从已有 npz 生成姿态分析图，跳过仿真')
    args = parser.parse_args()

    args.report_dir.mkdir(parents=True, exist_ok=True)
    DOCS_DIR.mkdir(parents=True, exist_ok=True)

    if args.plot_from_npz is not None:
        records = load_timeseries_npz(args.plot_from_npz)
        for mode_id, rec in records.items():
            plot_planning(rec, args.report_dir / f'mode_{mode_id}_planning.png')
            plot_control(rec, args.report_dir / f'mode_{mode_id}_control.png')
            if not args.skip_docs_copy:
                copy_to_docs(args.report_dir / f'mode_{mode_id}_planning.png',
                             f'mode_{mode_id}_planning.png')
                copy_to_docs(args.report_dir / f'mode_{mode_id}_control.png',
                             f'mode_{mode_id}_control.png')
        att_ts = args.report_dir / 'mode_attitude_error.png'
        att_psd = args.report_dir / 'mode_attitude_psd.png'
        att_rate = args.report_dir / 'mode_C_rpy_rate.png'
        plot_attitude_timeseries(records, att_ts)
        plot_attitude_psd(records, att_psd)
        plot_attitude_rpy(records, att_rate)
        if not args.skip_docs_copy:
            copy_to_docs(att_ts, 'mode_attitude_error.png')
            copy_to_docs(att_psd, 'mode_attitude_psd.png')
            copy_to_docs(att_rate, 'mode_C_rpy_rate.png')
        return

    all_stats = {}
    all_records: dict[str, ModeRecord] = {}

    for mode_id in args.modes:
        print(f'\n>>> 测试 {MODES[mode_id]["label"]} ...')
        record = record_mode(mode_id, MODES[mode_id]['label'])
        all_records[mode_id] = record
        stats = compute_stats(record)
        all_stats[mode_id] = stats

        plan_path = args.report_dir / f'mode_{mode_id}_planning.png'
        ctrl_path = args.report_dir / f'mode_{mode_id}_control.png'
        plot_planning(record, plan_path)
        plot_control(record, ctrl_path)

        if not args.skip_docs_copy:
            copy_to_docs(plan_path, f'mode_{mode_id}_planning.png')
            copy_to_docs(ctrl_path, f'mode_{mode_id}_control.png')

        print(f"  样本={stats['samples']} 位置均值={stats['position_norm_mm']['mean']:.3f}mm "
              f"角度均值={stats['orientation_norm_deg']['mean']:.4f}°")

    npz_path = args.report_dir / 'mode_test_timeseries.npz'
    save_timeseries_npz(all_records, npz_path)

    att_ts = args.report_dir / 'mode_attitude_error.png'
    att_psd = args.report_dir / 'mode_attitude_psd.png'
    att_rate = args.report_dir / 'mode_C_rpy_rate.png'
    plot_attitude_timeseries(all_records, att_ts)
    plot_attitude_psd(all_records, att_psd)
    plot_attitude_rpy(all_records, att_rate)
    if not args.skip_docs_copy:
        copy_to_docs(att_ts, 'mode_attitude_error.png')
        copy_to_docs(att_psd, 'mode_attitude_psd.png')
        copy_to_docs(att_rate, 'mode_C_rpy_rate.png')

    with (args.report_dir / 'mode_test_metrics.json').open('w', encoding='utf-8') as f:
        json.dump(all_stats, f, indent=2, ensure_ascii=False)

    write_summary_table(
        all_stats,
        args.report_dir / 'mode_test_report.md',
        args.report_dir / 'mode_test_metrics.csv',
    )
    print_table(all_stats)
    kill_stale()


if __name__ == '__main__':
    main()
