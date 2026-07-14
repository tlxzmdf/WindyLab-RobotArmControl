#!/usr/bin/env python3
"""优化版末端画圆节点。

相对 move_arm_ik_demo.py 的改进:
  - 100 Hz 发布，与 student_arm_node 控制周期对齐
  - 笛卡尔速度前馈 + 微分 IK（或离线预计算轨迹）
  - 发布关节速度，减轻 SmoothPositionController 限速滞后

用法:
    source /opt/ros/humble/setup.bash
    source windylab_ws/install/setup.bash
    python3 circle_draw_node.py [--mode diff|precompute]
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from _path_setup import _DEMO_DIR  # noqa: F401
from differential_ik import DifferentialIK, PrecomputedCircleTrajectory

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 100.0
CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
CIRCLE_RADIUS = 0.08
PERIOD_SEC = 8.0
# 与 student_arm_node SmoothPositionController max_velocity 对齐 (rad/s)
MAX_JOINT_VELOCITY = 0.35


def circle_kinematics(t: float) -> tuple[np.ndarray, np.ndarray]:
    """y-z 平面圆轨迹的位置与线速度 (base_link)。"""
    w = 2.0 * math.pi / PERIOD_SEC
    phase = w * t
    pos = CIRCLE_CENTER + CIRCLE_RADIUS * np.array([0.0, math.cos(phase), math.sin(phase)])
    vel = CIRCLE_RADIUS * w * np.array([0.0, -math.sin(phase), math.cos(phase)])
    return pos, vel


class CircleDrawNode(Node):
    def __init__(self, mode: str = 'diff', max_joint_velocity: float = MAX_JOINT_VELOCITY):
        super().__init__('circle_draw')
        self.mode = mode
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.max_joint_step = max_joint_velocity * self.dt
        self.t = 0.0
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)

        if mode == 'diff':
            self.tracker = DifferentialIK(pos_gain=10.0, max_dq=0.35)
            p0, _ = circle_kinematics(0.0)
            q0, ok = self.tracker.ik.solve(p0)
            if not ok:
                raise RuntimeError(f'start pose unreachable: {p0}')
            self.q = q0
            self.get_logger().info('mode=diff (differential IK + velocity feedforward)')
        elif mode == 'precompute':
            self.tracker = PrecomputedCircleTrajectory(
                CIRCLE_CENTER, CIRCLE_RADIUS, PERIOD_SEC, sample_hz=200.0)
            self.q, _ = self.tracker.sample(0.0)
            self.get_logger().info('mode=precompute (offline IK lookup)')
        else:
            raise ValueError(f'unknown mode: {mode}')

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'circle center={CIRCLE_CENTER.tolist()}, R={CIRCLE_RADIUS} m, '
            f'period={PERIOD_SEC}s, rate={PUBLISH_RATE_HZ} Hz, '
            f'max joint vel={max_joint_velocity} rad/s — Ctrl+C to stop')

    def tick(self) -> None:
        self.t += self.dt
        if self.mode == 'diff':
            target_pos, target_vel = circle_kinematics(self.t)
            step = self.tracker.step(
                self.q, target_pos, target_vel, self.dt,
                max_joint_step=self.max_joint_step)
            self.q = step.q
            dq = step.dq
            if not step.success and int(self.t * 10) % 50 == 0:
                self.get_logger().warn(
                    f'large EE error: {step.position_error * 1000:.1f} mm',
                    throttle_duration_sec=2.0)
        else:
            self.q, dq = self.tracker.sample(
                self.t, q_prev=self.q, max_joint_step=self.max_joint_step)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = self.q.tolist()
        msg.velocity = dq.tolist()
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='Optimized end-effector circle drawing')
    parser.add_argument(
        '--mode', choices=('diff', 'precompute'), default='diff',
        help='diff: online differential IK; precompute: offline trajectory lookup')
    parser.add_argument(
        '--max-joint-velocity', type=float, default=MAX_JOINT_VELOCITY,
        help='planner joint speed cap (rad/s), match student_arm max_velocity')
    args = parser.parse_args()

    rclpy.init()
    node = CircleDrawNode(mode=args.mode, max_joint_velocity=args.max_joint_velocity)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
