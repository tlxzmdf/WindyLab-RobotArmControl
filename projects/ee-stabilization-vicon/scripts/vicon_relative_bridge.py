#!/usr/bin/env python3
"""Vicon 飞机位姿 → 相对 t0 的机载扰动（供 ee-stabilization-vicon 使用）。

约定（标定完成后）:
  - /vrpn/<rigid>/pose 是飞机在 Vicon world 下的绝对位姿，零点不是机载端。
  - t0 锁定飞机位姿 T0；此后
        Δ(t) = inv(T0) * T(t)
    作为机载端相对扰动（平移 + 姿态）。
  - 稳定器将末端锁定在「启动时 / t0 附近」的世界系位姿；飞机相对 t0 的
    运动等价于机载端扰动。

输出:
  - TF: world → base_link   （真机 base_source:=tf）
  - /mount_disturbance/pose Float64MultiArray [x,y,z,roll,pitch,yaw]
        （仿真 base_source:=external；真机未订阅此话题）
  - /vicon_relative/delta PoseStamped 调试用
  - 服务 ~/latch_t0  重新冻结 t0

示例:
  ros2 run 不适用（非包内）；由 run_hw.sh / launch 调起:
  python3 vicon_relative_bridge.py --pose-topic /vrpn/pregme/pose

注意:
  run_hw.sh 默认 START_VRPN=true。claim_arm_serial 常会停掉 robot.service
  及其附带 VRPN；若 START_VRPN=false 且未另起 vicon_perception，本节点收不到
  pose，不会广播 world→base_link，稳定器 mount 恒为 0。
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def quat_xyzw(msg_q) -> np.ndarray:
    return np.array([msg_q.x, msg_q.y, msg_q.z, msg_q.w], dtype=float)


def rpy_from_quat_xyzw(q: np.ndarray) -> np.ndarray:
    """与 tf2 Matrix3x3::getRPY / EeStabilizationNode::QuatToRpy 一致。"""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)


def relative_delta(
    p0: np.ndarray, q0: np.ndarray, p: np.ndarray, q: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Δ = inv(T0)*T → (p_rel, q_rel_xyzw, rpy)."""
    R0 = Rotation.from_quat(q0)
    R = Rotation.from_quat(q)
    R_rel = R0.inv() * R
    p_rel = R0.inv().apply(p - p0)
    q_rel = R_rel.as_quat()  # xyzw
    # 归一化
    n = np.linalg.norm(q_rel)
    if n > 1e-12:
        q_rel = q_rel / n
    rpy = rpy_from_quat_xyzw(q_rel)
    return p_rel, q_rel, rpy


class ViconRelativeBridge(Node):
    def __init__(
        self,
        pose_topic: str,
        world_frame: str,
        base_frame: str,
        mount_base_offset_z: float,
        latch_delay_sec: float,
        publish_tf: bool,
        publish_mount_array: bool,
    ):
        super().__init__('vicon_relative_bridge')
        self.world_frame = world_frame
        self.base_frame = base_frame
        self.mount_base_offset_z = float(mount_base_offset_z)
        self.publish_tf = publish_tf
        self.publish_mount_array = publish_mount_array
        self.latch_delay_sec = float(latch_delay_sec)

        self._p0: Optional[np.ndarray] = None
        self._q0: Optional[np.ndarray] = None
        self._first_stamp: Optional[rclpy.time.Time] = None
        self._latched = False
        self._pose_topic = pose_topic
        self._pose_count = 0
        self._last_pose_wall: Optional[float] = None

        self._tf_broadcaster = TransformBroadcaster(self) if publish_tf else None
        self._mount_pub = (
            self.create_publisher(Float64MultiArray, '/mount_disturbance/pose', 10)
            if publish_mount_array
            else None
        )
        self._delta_pub = self.create_publisher(PoseStamped, '/vicon_relative/delta', 10)

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.create_service(Trigger, '~/latch_t0', self._on_latch_srv)
        # Diagnose missing VRPN: without pose we never publish TF → mount stays 0.
        self.create_timer(2.0, self._watchdog_pose)

        self.get_logger().info(
            f'Listening {pose_topic}; latch_delay={self.latch_delay_sec}s; '
            f'TF={publish_tf} ({world_frame}->{base_frame}); '
            f'mount_array={publish_mount_array}; offset_z={self.mount_base_offset_z}'
        )
        self.get_logger().info(
            'Δ = inv(T_plane(t0)) * T_plane(t)  as mount disturbance. '
            'Call service ~/latch_t0 to re-freeze t0.'
        )

    def _watchdog_pose(self) -> None:
        now = time.time()
        if self._pose_count == 0:
            self.get_logger().warn(
                f'No messages on {self._pose_topic} yet — TF {self.world_frame}->'
                f'{self.base_frame} will not appear. Is VRPN / vicon_perception running? '
                f'(run_hw.sh defaults START_VRPN=true; claim_arm_serial may have stopped '
                f'robot.service VRPN)'
            )
            return
        if self._last_pose_wall is not None and (now - self._last_pose_wall) > 2.0:
            self.get_logger().warn(
                f'{self._pose_topic} stalled for {now - self._last_pose_wall:.1f}s '
                f'(last count={self._pose_count}); check Vicon link'
            )

    def _on_latch_srv(self, _req, resp):
        self._p0 = None
        self._q0 = None
        self._first_stamp = None
        self._latched = False
        resp.success = True
        resp.message = 't0 cleared; will re-latch on next pose (after delay)'
        self.get_logger().info(resp.message)
        return resp

    def _maybe_latch(self, p: np.ndarray, q: np.ndarray, stamp) -> bool:
        now = self.get_clock().now()
        if self._first_stamp is None:
            self._first_stamp = now
            self.get_logger().info(
                f'Starting latch timer ({self.latch_delay_sec}s) … hold the aircraft still'
            )
        elapsed = (now - self._first_stamp).nanoseconds * 1e-9
        if elapsed < self.latch_delay_sec:
            return False
        self._p0 = p.copy()
        self._q0 = q.copy()
        self._latched = True
        self.get_logger().info(
            f'Latched t0 plane pose p={self._p0.tolist()} '
            f'rpy={rpy_from_quat_xyzw(self._q0).tolist()}'
        )
        return True

    def _on_pose(self, msg: PoseStamped):
        p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        q = quat_xyzw(msg.pose.orientation)
        n = np.linalg.norm(q)
        if n < 1e-9:
            return
        q = q / n
        self._pose_count += 1
        self._last_pose_wall = time.time()

        if not self._latched:
            if not self._maybe_latch(p, q, msg.header.stamp):
                # 未 latch 前发零扰动，便于稳定节点先锁末端
                self._publish_outputs(
                    np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), msg
                )
                return

        assert self._p0 is not None and self._q0 is not None
        p_rel, q_rel, rpy = relative_delta(self._p0, self._q0, p, q)
        self._publish_outputs(p_rel, q_rel, rpy, msg)

    def _publish_outputs(
        self, p_rel: np.ndarray, q_rel: np.ndarray, rpy: np.ndarray, src: PoseStamped
    ):
        # 调试 PoseStamped
        delta = PoseStamped()
        delta.header.stamp = src.header.stamp
        delta.header.frame_id = self.world_frame
        delta.pose.position.x = float(p_rel[0])
        delta.pose.position.y = float(p_rel[1])
        delta.pose.position.z = float(p_rel[2])
        delta.pose.orientation.x = float(q_rel[0])
        delta.pose.orientation.y = float(q_rel[1])
        delta.pose.orientation.z = float(q_rel[2])
        delta.pose.orientation.w = float(q_rel[3])
        self._delta_pub.publish(delta)

        if self._mount_pub is not None:
            arr = Float64MultiArray()
            arr.data = [
                float(p_rel[0]),
                float(p_rel[1]),
                float(p_rel[2]),
                float(rpy[0]),
                float(rpy[1]),
                float(rpy[2]),
            ]
            self._mount_pub.publish(arr)

        if self._tf_broadcaster is not None:
            # GetMountFromTf: T_world_drone from TF via T_world_base * T_base_drone
            # T_base_drone = translate(0,0,-offset)
            # Want T_world_drone = Δ  ⇒  T_world_base = Δ * translate(0,0,+offset)
            R_d = Rotation.from_quat(q_rel)
            t_base = p_rel + R_d.apply(np.array([0.0, 0.0, self.mount_base_offset_z]))
            q_base = q_rel

            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(t_base[0])
            tf.transform.translation.y = float(t_base[1])
            tf.transform.translation.z = float(t_base[2])
            tf.transform.rotation.x = float(q_base[0])
            tf.transform.rotation.y = float(q_base[1])
            tf.transform.rotation.z = float(q_base[2])
            tf.transform.rotation.w = float(q_base[3])
            self._tf_broadcaster.sendTransform(tf)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose-topic', default='/vrpn/pregme/pose')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--mount-base-offset-z', type=float, default=0.02)
    parser.add_argument(
        '--latch-delay',
        type=float,
        default=2.0,
        help='Seconds to wait after first pose before freezing t0 (hold plane still)',
    )
    parser.add_argument('--no-tf', action='store_true', help='Do not broadcast TF')
    parser.add_argument(
        '--no-mount-array',
        action='store_true',
        help='Do not publish /mount_disturbance/pose',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ViconRelativeBridge(
        pose_topic=args.pose_topic,
        world_frame=args.world_frame,
        base_frame=args.base_frame,
        mount_base_offset_z=args.mount_base_offset_z,
        latch_delay_sec=args.latch_delay,
        publish_tf=not args.no_tf,
        publish_mount_array=not args.no_mount_array,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
