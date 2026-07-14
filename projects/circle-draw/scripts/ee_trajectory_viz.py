#!/usr/bin/env python3
"""真机/仿真只读可视化：订阅 TF，不控制机械臂。

- 橙色小球：当前末端 link7（与 RViz 模型一致）
- 橙色轨迹线：delay 秒之前走过的路径（可选）

用法:
    python3 ee_trajectory_viz.py [--delay 10] [--trail-duration 10]
"""

from __future__ import annotations

import argparse
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

EE_FRAME = 'link7'
BASE_FRAME = 'base_link'
DEFAULT_DELAY_SEC = 10.0
DEFAULT_TRAIL_DURATION_SEC = 10.0
SAMPLE_RATE_HZ = 30.0


class EeTrajectoryViz(Node):
    def __init__(
        self,
        delay_sec: float = DEFAULT_DELAY_SEC,
        trail_duration_sec: float = DEFAULT_TRAIL_DURATION_SEC,
    ):
        super().__init__('ee_trajectory_viz')
        self.delay_sec = delay_sec
        self.trail_duration_sec = trail_duration_sec
        self.samples: deque[tuple[float, np.ndarray]] = deque()
        self._max_buffer_sec = delay_sec + trail_duration_sec + 2.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, '/hw_observe/ee_trail', 10)
        self.create_timer(1.0 / SAMPLE_RATE_HZ, self._on_timer)

        self.get_logger().info(
            f'observe-only EE viz: sphere=current {EE_FRAME}, '
            f'trail=delayed {delay_sec:.1f}s, frame={BASE_FRAME}')

    def _lookup_ee_position(self) -> tuple[float, np.ndarray] | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        t = tf_msg.header.stamp.sec + tf_msg.header.stamp.nanosec * 1e-9
        if t <= 1e-6:
            t = self.get_clock().now().nanoseconds * 1e-9
        p = tf_msg.transform.translation
        return t, np.array([p.x, p.y, p.z], dtype=float)

    def _record_sample(self, t: float, pos: np.ndarray) -> None:
        if self.samples and t < self.samples[-1][0] - 0.05:
            self.samples.clear()
        self.samples.append((t, pos))
        cutoff = t - self._max_buffer_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def _interp_position(self, t_query: float) -> np.ndarray | None:
        if not self.samples:
            return None
        if t_query <= self.samples[0][0]:
            return self.samples[0][1].copy()
        if t_query >= self.samples[-1][0]:
            return self.samples[-1][1].copy()
        for (t0, p0), (t1, p1) in zip(self.samples, list(self.samples)[1:]):
            if t0 <= t_query <= t1:
                if t1 <= t0:
                    return p0.copy()
                alpha = (t_query - t0) / (t1 - t0)
                return (1.0 - alpha) * p0 + alpha * p1
        return None

    def _collect_trail_points(self, t_end: float, t_start: float) -> list[np.ndarray]:
        points = [pos.copy() for t, pos in self.samples if t_start <= t <= t_end]
        if not points:
            return points
        p_start = self._interp_position(t_start)
        p_end = self._interp_position(t_end)
        if p_start is not None:
            points.insert(0, p_start)
        if p_end is not None:
            points.append(p_end)
        return points

    def _on_timer(self) -> None:
        looked_up = self._lookup_ee_position()
        if looked_up is None:
            return
        sample_t, current_pos = looked_up
        self._record_sample(sample_t, current_pos)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        t_end = now_sec - self.delay_sec
        t_start = t_end - self.trail_duration_sec
        stamp = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=0.5).to_msg()

        markers: list[Marker] = []

        if t_end > self.samples[0][0]:
            trail = Marker()
            trail.header.frame_id = BASE_FRAME
            trail.header.stamp = stamp
            trail.ns = 'hw_observe'
            trail.id = 0
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.004
            trail.color.r = 1.0
            trail.color.g = 0.55
            trail.color.b = 0.1
            trail.color.a = 0.85
            trail.lifetime = lifetime
            for pos in self._collect_trail_points(t_end, t_start):
                pt = Point()
                pt.x, pt.y, pt.z = float(pos[0]), float(pos[1]), float(pos[2])
                trail.points.append(pt)
            if trail.points:
                markers.append(trail)

        sphere = Marker()
        sphere.header.frame_id = BASE_FRAME
        sphere.header.stamp = stamp
        sphere.ns = 'hw_observe'
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(current_pos[0])
        sphere.pose.position.y = float(current_pos[1])
        sphere.pose.position.z = float(current_pos[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.018
        sphere.color.r = 1.0
        sphere.color.g = 0.45
        sphere.color.b = 0.05
        sphere.color.a = 0.9
        sphere.lifetime = lifetime
        markers.append(sphere)

        if markers:
            self.pub.publish(MarkerArray(markers=markers))


def main() -> None:
    parser = argparse.ArgumentParser(description='Read-only end-effector trail for hardware observe')
    parser.add_argument('--delay', type=float, default=DEFAULT_DELAY_SEC)
    parser.add_argument('--trail-duration', type=float, default=DEFAULT_TRAIL_DURATION_SEC)
    args = parser.parse_args()

    rclpy.init()
    node = EeTrajectoryViz(delay_sec=args.delay, trail_duration_sec=args.trail_duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
