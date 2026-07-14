#!/usr/bin/env python3
"""统一录制 CLIK / WBC 真机或仿真对比数据（无需 RViz）。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
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

ARM_ROOT = Path(__file__).resolve().parents[3]
WS = ARM_ROOT / 'windylab_ws'
DEFAULT_URDF = (
    WS / 'src' / 'arm_ee_stabilization_description' / 'urdf' / 'single_arm.urdf'
)
ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
JUMP_THRESH_RAD = 0.8


def _unwrap_delta(prev: float, cur: float) -> float:
    d = cur - prev
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


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


class CompareRecorder(Node):
    def __init__(
        self,
        out_dir: Path,
        method: str,
        urdf_path: Path,
        live_interval: float,
    ) -> None:
        super().__init__('compare_recorder')
        self.out_dir = out_dir
        self.method = method
        self.live_interval = live_interval
        self.t0: Optional[float] = None
        self.last_live = 0.0
        out_dir.mkdir(parents=True, exist_ok=True)

        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.ee_fid = self.model.getFrameId('link6')

        self.task_rows: list[dict] = []
        self.teleop_rows: list[dict] = []
        self.slave_rows: list[dict] = []
        self.master_rows: list[dict] = []
        self.jump_rows: list[dict] = []
        self._last_master_q: Optional[list[float]] = None
        self._last_slave_q: Optional[list[float]] = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        err_topic = (
            '/stabilization_error' if method == 'clik' else '/teleop_wbc_error'
        )
        self.create_subscription(JointState, '/master/joint_states', self._on_master, 50)
        self.create_subscription(JointState, '/joint_states', self._on_slave, 50)
        self.create_subscription(Float64MultiArray, err_topic, self._on_err, 50)

        self.live_path = out_dir / 'live_log.txt'
        self.live_path.write_text('', encoding='utf-8')
        self.get_logger().info(f'method={method} out={out_dir} err={err_topic}')

    def _rel_t(self, stamp: float) -> float:
        if self.t0 is None:
            self.t0 = stamp
        return stamp - self.t0

    def _on_master(self, msg: JointState) -> None:
        q = []
        for name in ARM_JOINTS:
            if name not in msg.name:
                return
            idx = msg.name.index(name)
            q.append(float(msg.position[idx]))
        t = self._rel_t(
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        self.master_rows.append({'t_sec': round(t, 6), **{
            f'{n}_pos': q[i] for i, n in enumerate(ARM_JOINTS)
        }})
        self._last_master_q = q

    def _on_slave(self, msg: JointState) -> None:
        q = []
        for name in ARM_JOINTS:
            if name not in msg.name:
                return
            idx = msg.name.index(name)
            q.append(float(msg.position[idx]))
        t = self._rel_t(
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        if self._last_slave_q is not None:
            for i, name in enumerate(ARM_JOINTS):
                delta = abs(_unwrap_delta(self._last_slave_q[i], q[i]))
                if delta >= JUMP_THRESH_RAD:
                    self.jump_rows.append({
                        't_sec': round(t, 6),
                        'joint': name,
                        'delta_deg': round(math.degrees(delta), 3),
                    })
        self._last_slave_q = q
        row = {'t_sec': round(t, 6)}
        for i, name in enumerate(ARM_JOINTS):
            row[f'{name}_pos'] = q[i]
            if self._last_master_q:
                row[f'{name}_err'] = _unwrap_delta(self._last_master_q[i], q[i])
        self.slave_rows.append(row)

    def _on_err(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return
        t = self._rel_t(self.get_clock().now().nanoseconds * 1e-9)
        row = {
            't_sec': round(t, 6),
            'world_pos_err_m': float(msg.data[0]),
            'world_orient_err_rad': float(msg.data[1]),
            'base_pos_err_m': float(msg.data[2]),
            'base_orient_err_rad': float(msg.data[3]),
        }
        if self.method == 'clik' and len(msg.data) >= 5:
            row['solve_time_us'] = float(msg.data[4])
        elif self.method == 'wbc' and len(msg.data) >= 15:
            row['solve_time_us'] = float(msg.data[14])
        self.task_rows.append(row)

        if self._last_master_q is None:
            return
        q = pin.neutral(self.model)
        for i, v in enumerate(self._last_master_q):
            if i < self.model.nq:
                q[i] = v
        pin.framesForwardKinematics(self.model, self.data, q)
        m_pos = self.data.oMf[self.ee_fid].translation.copy()
        try:
            tf = self._tf_buffer.lookup_transform('world', 'link6', rclpy.time.Time())
            s_pos = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ])
            self.teleop_rows.append({
                't_sec': round(t, 6),
                'master_slave_pos_err_m': float(np.linalg.norm(m_pos - s_pos)),
            })
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        now = time.time()
        if now - self.last_live >= self.live_interval:
            w_mm = row['world_pos_err_m'] * 1000.0
            st = row.get('solve_time_us', 0.0)
            line = f'[{t:5.1f}s] world={w_mm:.2f}mm solve={st:.0f}us'
            print(line, flush=True)
            with self.live_path.open('a', encoding='utf-8') as f:
                f.write(line + '\n')
            self.last_live = now


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    keys: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for k in row:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def save_all(out_dir: Path, node: CompareRecorder, duration: float, meta: dict) -> None:
    _write_csv(out_dir / 'task_error.csv', node.task_rows)
    _write_csv(out_dir / 'teleop_tracking.csv', node.teleop_rows)
    _write_csv(out_dir / 'slave_joints.csv', node.slave_rows)
    _write_csv(out_dir / 'master_joints.csv', node.master_rows)
    _write_csv(out_dir / 'joint_jumps.csv', node.jump_rows)

    w_pos = [r['world_pos_err_m'] * 1000 for r in node.task_rows]
    solve = [r.get('solve_time_us', 0.0) for r in node.task_rows if 'solve_time_us' in r]
    ms_pos = [r['master_slave_pos_err_m'] * 1000 for r in node.teleop_rows]

    summary = {
        'method': node.method,
        'duration_sec': duration,
        'samples': len(node.task_rows),
        'world_pos_mm': _stats(w_pos),
        'master_slave_pos_mm': _stats(ms_pos),
        'solve_time_us': _stats(solve),
        'jump_count': len(node.jump_rows),
    }
    (out_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    meta.update(summary)
    (out_dir / 'run_meta.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', choices=['clik', 'wbc'], required=True)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--live-interval', type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = CompareRecorder(args.out, args.method, args.urdf, args.live_interval)
    meta = {
        'recorded_at': datetime.now(timezone.utc).isoformat(),
        'compare_config': 'teleop_compare_mode_b.yaml',
    }
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        save_all(args.out, node, args.duration, meta)
        node.destroy_node()
        rclpy.try_shutdown()

    if len(node.task_rows) < 30:
        print('[WARN] 样本过少', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
