#!/usr/bin/env python3
"""驱动真机主臂（student_arm）并同步遥操作目标。

真机默认处于位置控制，无法手拖；本脚本向 /student/joint_command 发轨迹，
从臂通过 /joint_states 跟踪（launch 中 master_mode:=position）。
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f'joint{i}' for i in range(1, 8)]
CMD_JOINTS = ARM_JOINTS
HOME = [0.0, 0.35, -0.55, 0.0, 0.45, 0.0, 0.0]


class MasterCommandDemo(Node):
    def __init__(self) -> None:
        super().__init__('master_command_demo')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('amplitude', 0.15)
        self.declare_parameter('max_velocity', 0.25)
        self._t0 = self.get_clock().now()
        rate = float(self.get_parameter('rate_hz').value)
        self._max_vel = float(self.get_parameter('max_velocity').value)
        self._pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            '真机主臂轨迹 demo → /student/joint_command '
            f'(amp={self.get_parameter("amplitude").value}, max_vel={self._max_vel})')

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        amp = float(self.get_parameter('amplitude').value)
        q = list(HOME)
        q[1] += amp * math.sin(0.30 * t)
        q[2] += amp * 0.55 * math.cos(0.25 * t)
        q[4] += amp * 0.45 * math.sin(0.20 * t + 0.4)
        dq = [
            0.0,
            amp * 0.30 * math.cos(0.30 * t),
            -amp * 0.55 * 0.25 * math.sin(0.25 * t),
            0.0,
            amp * 0.45 * 0.20 * math.cos(0.20 * t + 0.4),
            0.0,
            0.0,
        ]
        for i in range(len(dq)):
            dq[i] = max(-self._max_vel, min(self._max_vel, dq[i]))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = CMD_JOINTS
        msg.position = q
        msg.velocity = dq
        self._pub.publish(msg)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = MasterCommandDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
