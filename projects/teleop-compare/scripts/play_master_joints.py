#!/usr/bin/env python3
"""回放 master_joints.csv → /master/joint_states（与手拖轨迹一致）。"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]


def _row_positions(row: dict) -> list[float]:
    pos = []
    for name in ARM_JOINTS:
        if f'{name}_pos' in row:
            pos.append(float(row[f'{name}_pos']))
        else:
            pos.append(float(row[name]))
    return pos


class MasterJointPlayer(Node):
    def __init__(self, csv_path: Path, rate_hz: float) -> None:
        super().__init__('master_joint_player')
        self.rows = list(csv.DictReader(csv_path.open(encoding='utf-8')))
        if not self.rows:
            raise RuntimeError(f'空轨迹文件: {csv_path}')
        self.idx = 0
        self._pub = self.create_publisher(JointState, '/master/joint_states', 10)
        self._t0 = time.monotonic()
        self._duration = float(self.rows[-1]['t_sec'])
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'回放 {csv_path.name}: {len(self.rows)} 样本, {self._duration:.2f}s')

    def _tick(self) -> None:
        if self.idx >= len(self.rows):
            return
        elapsed = time.monotonic() - self._t0
        while self.idx < len(self.rows) and float(self.rows[self.idx]['t_sec']) <= elapsed:
            row = self.rows[self.idx]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_JOINTS
            msg.position = _row_positions(row)
            msg.velocity = [0.0] * 6
            self._pub.publish(msg)
            self.idx += 1

    @property
    def duration_sec(self) -> float:
        return self._duration


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    parser.add_argument('--rate-hz', type=float, default=100.0)
    parser.add_argument('--pad-sec', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = MasterJointPlayer(args.csv, args.rate_hz)
    end = time.monotonic() + node.duration_sec + args.pad_sec
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
