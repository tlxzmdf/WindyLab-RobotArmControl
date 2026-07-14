#!/usr/bin/env python3
"""仿真主臂运动 demo：向 /master/joint_states 发布关节轨迹。

用于无真机时验证 master-slave-stabilization 项目。
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
HOME = [0.0, 0.35, -0.55, 0.0, 0.45, 0.0]


class MasterMotionDemo(Node):
    def __init__(self) -> None:
        super().__init__('master_motion_demo')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('amplitude', 0.18)
        self._t0 = self.get_clock().now()
        rate = self.get_parameter('rate_hz').value
        self._pub = self.create_publisher(JointState, '/master/joint_states', 10)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        amp = float(self.get_parameter('amplitude').value)
        q = list(HOME)
        q[1] += amp * math.sin(0.35 * t)
        q[2] += amp * 0.6 * math.cos(0.28 * t)
        q[4] += amp * 0.5 * math.sin(0.22 * t + 0.5)
        dq = [
            0.0,
            amp * 0.35 * math.cos(0.35 * t),
            -amp * 0.6 * 0.28 * math.sin(0.28 * t),
            0.0,
            amp * 0.5 * 0.22 * math.cos(0.22 * t + 0.5),
            0.0,
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS
        msg.position = q
        msg.velocity = dq
        self._pub.publish(msg)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = MasterMotionDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
