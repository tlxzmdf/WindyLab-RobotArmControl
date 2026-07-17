#!/usr/bin/env python3
"""Smoothly move the student arm to nominal home (q_home) before stabilization tests.

Publishes /student/joint_command at a fixed rate while reading /joint_states.
Uses cosine interpolation for a gentle trajectory.

Default q_home matches arm_ee_stabilization_control/config/stabilization.yaml:
  [0.0, 0.35, -0.55, 0.0, 0.45, 0.0] (+ joint7 = 0)
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_Q_HOME = [0.0, 0.35, -0.55, 0.0, 0.45, 0.0, 0.0]
JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]


def _extract_q(msg: JointState, n: int) -> Optional[list[float]]:
    name_to_idx = {name: i for i, name in enumerate(msg.name)}
    q: list[float] = []
    for i in range(n):
        name = JOINT_NAMES[i]
        if name not in name_to_idx:
            return None
        idx = name_to_idx[name]
        if idx >= len(msg.position):
            return None
        q.append(float(msg.position[idx]))
    return q


class MoveToHome(Node):
    def __init__(
        self,
        q_home: list[float],
        duration: float,
        rate_hz: float,
        settle: float,
        tol: float,
    ) -> None:
        super().__init__("move_to_home")
        self.q_home = q_home
        self.n = len(q_home)
        self.duration = max(0.5, duration)
        self.dt = 1.0 / max(rate_hz, 1.0)
        self.settle = max(0.0, settle)
        self.tol = tol

        self.q_start: Optional[list[float]] = None
        self.q_meas: Optional[list[float]] = None
        self.t0: Optional[float] = None
        self.done = False

        self.pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_js, 50
        )
        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f"Waiting for /joint_states then moving to home in {self.duration:.1f}s: "
            f"{[round(x, 3) for x in self.q_home]}"
        )

    def _on_js(self, msg: JointState) -> None:
        q = _extract_q(msg, self.n)
        if q is None:
            return
        self.q_meas = q
        if self.q_start is None:
            self.q_start = list(q)
            self.t0 = time.time()
            self.get_logger().info(
                f"Start q={[round(x, 3) for x in self.q_start]} -> home"
            )

    def _on_timer(self) -> None:
        if self.done or self.q_start is None or self.t0 is None:
            return

        elapsed = time.time() - self.t0
        if elapsed <= self.duration:
            # cosine ease-in-out: s in [0,1]
            u = elapsed / self.duration
            s = 0.5 - 0.5 * math.cos(math.pi * u)
        else:
            s = 1.0

        q_cmd = [
            self.q_start[i] + s * (self.q_home[i] - self.q_start[i])
            for i in range(self.n)
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES[: self.n]
        msg.position = q_cmd
        msg.velocity = [0.0] * self.n
        msg.effort = [0.0] * self.n
        self.pub.publish(msg)

        if elapsed >= self.duration + self.settle:
            if self.q_meas is not None:
                err = max(abs(self.q_meas[i] - self.q_home[i]) for i in range(self.n))
                self.get_logger().info(
                    f"Home done. max_|q-q_home|={err:.4f} rad (tol={self.tol})"
                )
                if err > self.tol:
                    self.get_logger().warn(
                        "Home tolerance not met; continuing launch anyway"
                    )
            else:
                self.get_logger().warn("No joint feedback at end of home")
            self.done = True


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=6.0, help="Motion duration (s)")
    p.add_argument("--settle", type=float, default=0.8, help="Hold at home (s)")
    p.add_argument("--rate", type=float, default=50.0, help="Command rate (Hz)")
    p.add_argument("--tol", type=float, default=0.08, help="Max |q-q_home| warn tol")
    p.add_argument(
        "--q-home",
        type=str,
        default=",".join(str(x) for x in DEFAULT_Q_HOME),
        help="Comma-separated home joints (6 or 7)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Fail if no joint_states before this many seconds",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    q_home = [float(x) for x in args.q_home.split(",") if x.strip() != ""]
    if len(q_home) == 6:
        q_home.append(0.0)
    if len(q_home) != 7:
        print("q-home must have 6 or 7 values", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveToHome(
        q_home=q_home,
        duration=args.duration,
        rate_hz=args.rate,
        settle=args.settle,
        tol=args.tol,
    )
    t_start = time.time()
    wait_js_deadline = t_start + max(5.0, args.timeout * 0.5)
    deadline = t_start + max(args.timeout, args.duration + args.settle + 10.0)
    try:
        while rclpy.ok() and not node.done:
            now = time.time()
            if now > deadline:
                node.get_logger().error("move_to_home timed out")
                return 1
            if node.q_start is None and now > wait_js_deadline:
                node.get_logger().error(
                    "No /joint_states yet — is student_arm_node running?"
                )
                return 1
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.done else 1


if __name__ == "__main__":
    raise SystemExit(main())
