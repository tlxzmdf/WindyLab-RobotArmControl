#!/usr/bin/env python3
"""发布机载端扰动位姿到 /mount_disturbance/pose（配合 base_source:=external）。"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MountDisturbanceProfile(Node):
    def __init__(self) -> None:
        super().__init__('mount_disturbance_profile')
        self.declare_parameter('profile', 'sway')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('roll_amp_deg', 10.0)
        self.declare_parameter('pitch_amp_deg', 15.0)
        self.declare_parameter('sway_period_sec', 4.0)
        self.declare_parameter('pitch_step_deg', 20.0)
        self.declare_parameter('pitch_step_time_sec', 8.0)
        self._t0 = self.get_clock().now()
        self._pub = self.create_publisher(Float64MultiArray, '/mount_disturbance/pose', 10)
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

    def _sample_rpy(self, t: float) -> tuple[float, float, float]:
        profile = str(self.get_parameter('profile').value)
        if profile == 'idle':
            return 0.0, 0.0, 0.0
        if profile == 'pitch_step':
            step_t = float(self.get_parameter('pitch_step_time_sec').value)
            step_deg = float(self.get_parameter('pitch_step_deg').value)
            pitch = math.radians(step_deg) if t >= step_t else 0.0
            return 0.0, pitch, 0.0
        period = max(float(self.get_parameter('sway_period_sec').value), 0.5)
        roll_amp = math.radians(float(self.get_parameter('roll_amp_deg').value))
        pitch_amp = math.radians(float(self.get_parameter('pitch_amp_deg').value))
        roll = roll_amp * math.sin(2.0 * math.pi * t / period)
        pitch = pitch_amp * math.sin(2.0 * math.pi * t / (period * 1.27) + 0.6)
        yaw = 0.5 * roll_amp * math.sin(2.0 * math.pi * t / (period * 1.9) + 1.1)
        return roll, pitch, yaw

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        roll, pitch, yaw = self._sample_rpy(t)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, roll, pitch, yaw]
        self._pub.publish(msg)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = MountDisturbanceProfile()
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
