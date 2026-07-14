#!/usr/bin/env python3
"""主从末端自稳 — 30s 运行数据录制（实时统计 + CSV 落盘）。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
import pinocchio as pin
import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
WS = ARM_ROOT / 'windylab_ws'
DEFAULT_URDF = WS / 'src' / 'arm-platform' / 'config' / 'arm.urdf'

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
MOUNT_JOINTS = [
    'mount_tx', 'mount_ty', 'mount_tz',
    'mount_rx', 'mount_ry', 'mount_rz',
]
EE_FRAME = 'link6'


def _stamp_sec(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _extract_named(msg: JointState, names: list[str]) -> Optional[list[float]]:
    vals = []
    for name in names:
        if name not in msg.name:
            return None
        idx = msg.name.index(name)
        vals.append(float(msg.position[idx]) if idx < len(msg.position) else 0.0)
    return vals


@dataclass
class RollingStats:
    values: list[float] = field(default_factory=list)

    def add(self, v: float) -> None:
        self.values.append(v)

    def mean(self) -> float:
        return float(np.mean(self.values)) if self.values else 0.0

    def rms(self) -> float:
        if not self.values:
            return 0.0
        a = np.array(self.values)
        return float(np.sqrt(np.mean(a * a)))

    def max(self) -> float:
        return float(np.max(self.values)) if self.values else 0.0

    def clear(self) -> None:
        self.values.clear()


class TeleopRunRecorder(Node):
    def __init__(self, out_dir: Path, urdf_path: Path, live_interval: float) -> None:
        super().__init__('teleop_run_recorder')
        self.out_dir = out_dir
        self.live_interval = live_interval
        self.t0: Optional[float] = None
        self.last_live = 0.0

        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.ee_fid = self.model.getFrameId(EE_FRAME)

        self.master_rows: list[dict] = []
        self.slave_rows: list[dict] = []
        self.mount_rows: list[dict] = []
        self.ee_err_rows: list[dict] = []
        self.teleop_rows: list[dict] = []
        self.ref_rows: list[dict] = []

        self._last_master_q: Optional[list[float]] = None
        self._last_slave_ee: Optional[np.ndarray] = None

        self.roll_pos = RollingStats()
        self.roll_orient = RollingStats()
        self.roll_teleop_pos = RollingStats()
        self.roll_teleop_orient = RollingStats()
        self.sample_count = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.live_path = out_dir / 'live_log.txt'
        self.live_path.write_text('', encoding='utf-8')

        self.create_subscription(JointState, '/master/joint_states', self._on_master, 50)
        self.create_subscription(JointState, '/joint_states', self._on_slave, 50)
        self.create_subscription(Float64MultiArray, '/stabilization_error', self._on_err, 50)
        self.create_subscription(JointState, '/stabilization_reference', self._on_ref, 50)

        self.get_logger().info(f'录制目录: {out_dir}')

    def _rel_t(self, stamp: float) -> float:
        if self.t0 is None:
            self.t0 = stamp
        return stamp - self.t0

    def _master_ee(self, q6: list[float]) -> tuple[np.ndarray, np.ndarray]:
        q = pin.neutral(self.model)
        for i, val in enumerate(q6):
            if i < self.model.nq:
                q[i] = val
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.ee_fid]
        return oMf.translation.copy(), oMf.rotation.copy()

    def _slave_ee_world(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        try:
            tf = self._tf_buffer.lookup_transform('world', EE_FRAME, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = tf.transform.translation
        qr = tf.transform.rotation
        pos = np.array([tr.x, tr.y, tr.z])
        rot = pin.Quaternion(qr.w, qr.x, qr.y, qr.z).matrix()
        return pos, rot

    @staticmethod
    def _orient_err(r_act: np.ndarray, r_des: np.ndarray) -> float:
        return float(np.linalg.norm(pin.log3(r_act.T @ r_des)))

    def _on_master(self, msg: JointState) -> None:
        q = _extract_named(msg, ARM_JOINTS)
        if q is None:
            return
        stamp = _stamp_sec(msg)
        t = self._rel_t(stamp)
        row = {'t_sec': round(t, 6)}
        for i, name in enumerate(ARM_JOINTS):
            row[f'{name}_pos'] = q[i]
        self.master_rows.append(row)
        self._last_master_q = q

    def _on_slave(self, msg: JointState) -> None:
        q = _extract_named(msg, ARM_JOINTS)
        m = _extract_named(msg, MOUNT_JOINTS)
        if q is None:
            return
        stamp = _stamp_sec(msg)
        t = self._rel_t(stamp)
        srow = {'t_sec': round(t, 6)}
        for i, name in enumerate(ARM_JOINTS):
            srow[f'{name}_pos'] = q[i]
        self.slave_rows.append(srow)
        if m is not None:
            mrow = {'t_sec': round(t, 6)}
            for i, name in enumerate(MOUNT_JOINTS):
                mrow[f'{name}_pos'] = m[i]
            self.mount_rows.append(mrow)

    def _on_err(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return
        stamp = self.get_clock().now().nanoseconds * 1e-9
        t = self._rel_t(stamp)
        w_pos, w_ori, b_pos, b_ori = [float(x) for x in msg.data[:4]]
        self.ee_err_rows.append({
            't_sec': round(t, 6),
            'world_pos_err_m': w_pos,
            'world_orient_err_rad': w_ori,
            'base_pos_err_m': b_pos,
            'base_orient_err_rad': b_ori,
        })
        self.roll_pos.add(w_pos * 1000.0)
        self.roll_orient.add(w_ori)
        self.sample_count += 1

        if self._last_master_q is not None:
            m_pos, m_rot = self._master_ee(self._last_master_q)
            slave = self._slave_ee_world()
            if slave is not None:
                s_pos, s_rot = slave
                t_pos = float(np.linalg.norm(m_pos - s_pos))
                t_ori = self._orient_err(s_rot, m_rot)
                self.teleop_rows.append({
                    't_sec': round(t, 6),
                    'master_slave_pos_err_m': t_pos,
                    'master_slave_orient_err_rad': t_ori,
                })
                self.roll_teleop_pos.add(t_pos * 1000.0)
                self.roll_teleop_orient.add(t_ori)

        now = time.time()
        if now - self.last_live >= self.live_interval:
            self._flush_live(t, w_pos, w_ori)
            self.last_live = now

    def _on_ref(self, msg: JointState) -> None:
        q_plan = _extract_named(msg, ARM_JOINTS)
        if q_plan is None:
            return
        stamp = _stamp_sec(msg)
        t = self._rel_t(stamp)
        row = {'t_sec': round(t, 6)}
        for i, name in enumerate(ARM_JOINTS):
            row[f'{name}_plan'] = q_plan[i]
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.velocity):
                    row[f'{name}_cmd'] = float(msg.velocity[idx])
        self.ref_rows.append(row)

    def _flush_live(self, t: float, w_pos: float, w_ori: float) -> None:
        line = (
            f'[{t:5.1f}s] samples={self.sample_count} '
            f'EE自稳 pos RMS={self.roll_pos.rms():.3f}mm max={self.roll_pos.max():.3f}mm | '
            f'orient RMS={self.roll_orient.rms()*1000:.2f}mrad | '
            f'主从跟踪 pos RMS={self.roll_teleop_pos.rms():.3f}mm max={self.roll_teleop_pos.max():.3f}mm | '
            f'orient RMS={self.roll_teleop_orient.rms()*1000:.2f}mrad | '
            f'瞬时 world_err={w_pos*1000:.2f}mm'
        )
        print(line, flush=True)
        with self.live_path.open('a', encoding='utf-8') as f:
            f.write(line + '\n')
        self.roll_pos.clear()
        self.roll_orient.clear()
        self.roll_teleop_pos.clear()
        self.roll_teleop_orient.clear()


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _stats(values: list[float]) -> dict:
    if not values:
        return {'mean': 0.0, 'rms': 0.0, 'max': 0.0, 'p95': 0.0}
    a = np.array(values)
    return {
        'mean': float(np.mean(a)),
        'rms': float(np.sqrt(np.mean(a * a))),
        'max': float(np.max(a)),
        'p95': float(np.percentile(a, 95)),
    }


def write_summary(out_dir: Path, duration: float, ee_rows: list[dict], teleop_rows: list[dict]) -> None:
    w_pos = [r['world_pos_err_m'] * 1000 for r in ee_rows]
    w_ori = [r['world_orient_err_rad'] * 1000 for r in ee_rows]
    t_pos = [r['master_slave_pos_err_m'] * 1000 for r in teleop_rows]
    t_ori = [r['master_slave_orient_err_rad'] * 1000 for r in teleop_rows]

    sp = _stats(w_pos)
    so = _stats(w_ori)
    tp = _stats(t_pos)
    to = _stats(t_ori)

    lines = [
        'master-slave-stabilization 录制摘要',
        f'duration={duration:.1f}s  ee_samples={len(ee_rows)}  teleop_samples={len(teleop_rows)}',
        '',
        '末端自稳 (world EE vs 目标, mm / mrad):',
        f'  position  mean={sp["mean"]:.3f}  rms={sp["rms"]:.3f}  max={sp["max"]:.3f}  p95={sp["p95"]:.3f}',
        f'  orient    mean={so["mean"]:.3f}  rms={so["rms"]:.3f}  max={so["max"]:.3f}  p95={so["p95"]:.3f}',
        '',
        '主从跟踪 (master EE vs slave EE, mm / mrad):',
        f'  position  mean={tp["mean"]:.3f}  rms={tp["rms"]:.3f}  max={tp["max"]:.3f}  p95={tp["p95"]:.3f}',
        f'  orient    mean={to["mean"]:.3f}  rms={to["rms"]:.3f}  max={to["max"]:.3f}  p95={to["p95"]:.3f}',
        '',
        '调参建议:',
        '  · 主从滞后大 → 降低 teleop_target_filter 或提高 q_des_filter',
        '  · 自稳误差大 → 增大 kp_task / joint_kp，或减小 disturbance_radius',
    ]
    (out_dir / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n' + (out_dir / 'summary.txt').read_text(encoding='utf-8'))


def save_all(out_dir: Path, node: TeleopRunRecorder, duration: float, meta: dict) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(out_dir / 'master_joints.csv', node.master_rows)
    _write_csv(out_dir / 'slave_joints.csv', node.slave_rows)
    _write_csv(out_dir / 'mount_pose.csv', node.mount_rows)
    _write_csv(out_dir / 'ee_stabilization_error.csv', node.ee_err_rows)
    _write_csv(out_dir / 'teleop_tracking_error.csv', node.teleop_rows)
    _write_csv(out_dir / 'stabilization_reference.csv', node.ref_rows)
    meta.update({
        'master_samples': len(node.master_rows),
        'slave_samples': len(node.slave_rows),
        'ee_error_samples': len(node.ee_err_rows),
        'teleop_samples': len(node.teleop_rows),
    })
    (out_dir / 'run_meta.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    write_summary(out_dir, duration, node.ee_err_rows, node.teleop_rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--live-interval', type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = TeleopRunRecorder(args.out, args.urdf, args.live_interval)
    meta = {
        'project': 'master-slave-stabilization',
        'duration_sec': args.duration,
        'recorded_at': datetime.now(timezone.utc).isoformat(),
        'topics': [
            '/master/joint_states',
            '/joint_states',
            '/stabilization_error',
            '/stabilization_reference',
        ],
    }
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        save_all(args.out, node, args.duration, meta)
        node.destroy_node()
        rclpy.try_shutdown()

    if len(node.ee_err_rows) < 50:
        print('[WARN] 样本过少，请确认 launch 与 master_motion_demo 已运行', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
