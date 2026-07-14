#!/usr/bin/env python3
"""让 link5 向右转 90°（joint5 增加 π/2 rad）。

从当前关节角出发，仅改变 joint5，其余关节保持不动；余弦插值，端点速度为零。
仿真与真机共用本脚本，只需 launch 时切换 arm_type。

用法:
    # 终端 1 — 仿真
    ros2 launch manipulator student_arm.launch.py arm_type:=sim

    # 终端 1 — 真机
    ros2 launch manipulator student_arm.launch.py arm_type:=a_l1 port_name:=/dev/ttyUSB0 max_velocity:=0.2

    # 终端 2
    source /opt/ros/humble/setup.bash && source install/setup.bash
    cd src/arm-platform/demo
    python3 rotate_link5_right_90.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_COUNT = 7
JOINT5_INDEX = 4
DELTA_RAD = math.pi / 2.0
JOINT5_LOWER = -2.0
JOINT5_UPPER = 2.0
PUBLISH_RATE_HZ = 50.0
MOTION_SEC = 4.0
HOLD_SEC = 1.0
WAIT_FEEDBACK_SEC = 5.0


def cosine_interp(s: float, q0: float, q1: float) -> tuple[float, float]:
    """s ∈ [0, 1]，端点速度为零。"""
    alpha = 0.5 * (1.0 - math.cos(math.pi * s))
    q = q0 + alpha * (q1 - q0)
    if MOTION_SEC <= 0.0:
        return q1, 0.0
    dq_ds = 0.5 * math.pi * math.sin(math.pi * s)
    dq_dt = dq_ds * (q1 - q0) / MOTION_SEC
    return q, dq_dt


class RotateLink5Right90(Node):
    def __init__(self):
        super().__init__('rotate_link5_right_90')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.current_q = None
        self.target_q = None
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self.get_logger().info('等待 /joint_states 反馈...')

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.position) != JOINT_COUNT:
            return
        if self.current_q is None:
            self.current_q = list(msg.position)

    def wait_for_feedback(self) -> bool:
        deadline = time.time() + WAIT_FEEDBACK_SEC
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_q is not None:
                return True
        return False

    def publish_command(self, q: list[float], dq: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = q
        msg.velocity = dq
        self.pub.publish(msg)

    def run_motion(self) -> int:
        if not self.wait_for_feedback():
            self.get_logger().error('未收到 /joint_states，请先启动 student_arm.launch.py')
            return 1

        q_start = self.current_q[:]
        q_goal = q_start[:]
        q_goal[JOINT5_INDEX] += DELTA_RAD

        if q_goal[JOINT5_INDEX] > JOINT5_UPPER:
            self.get_logger().error(
                f'joint5 目标 {q_goal[JOINT5_INDEX]:.3f} rad 超出上限 {JOINT5_UPPER} rad，'
                f'当前 {q_start[JOINT5_INDEX]:.3f} rad')
            return 1

        self.get_logger().info(
            f'joint5: {q_start[JOINT5_INDEX]:.3f} -> {q_goal[JOINT5_INDEX]:.3f} rad '
            f'(+90°), 用时 {MOTION_SEC:.1f}s')

        dt = 1.0 / PUBLISH_RATE_HZ
        t0 = time.time()

        while rclpy.ok():
            elapsed = time.time() - t0
            if elapsed <= MOTION_SEC:
                s = min(1.0, elapsed / MOTION_SEC)
                q5, dq5 = cosine_interp(s, q_start[JOINT5_INDEX], q_goal[JOINT5_INDEX])
            elif elapsed <= MOTION_SEC + HOLD_SEC:
                q5, dq5 = q_goal[JOINT5_INDEX], 0.0
            else:
                break

            q = q_start[:]
            q[JOINT5_INDEX] = q5
            dq = [0.0] * JOINT_COUNT
            dq[JOINT5_INDEX] = dq5
            self.publish_command(q, dq)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.get_logger().info('完成：link5 已右转 90°')
        return 0


def main():
    rclpy.init()
    node = RotateLink5Right90()
    try:
        code = node.run_motion()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
