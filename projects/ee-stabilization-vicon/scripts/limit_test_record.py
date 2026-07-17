#!/usr/bin/env python3
"""ROS2 recording for EE stabilization limit tests (isolated from analysis path)."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from limit_test_lib import (
    ARM_JOINTS,
    LAUNCH_FILE,
    MODES,
    MOUNT_JOINTS,
    NOMINAL_ORIENT_AMP,
    NOMINAL_RADIUS,
    NOMINAL_TIME_CONSTANT,
    RECORD_SEC,
    WARMUP_SEC,
    WS,
    DisturbanceCase,
    RunRecord,
    Sample,
    compute_metrics,
)


def kill_stale() -> None:
    patterns = [
        "ee_stabilization",
        "limit_test_headless.launch",
        "stabilization_headless.launch",
        "stabilization.launch",
        "robot_state_publisher.*arm_on_drone",
    ]
    for pat in patterns:
        subprocess.run(["pkill", "-f", pat], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def _bool_str(value: bool) -> str:
    return "true" if value else "false"


def launch_case(case: DisturbanceCase) -> subprocess.Popen:
    if not LAUNCH_FILE.exists():
        raise FileNotFoundError(f"Launch file missing: {LAUNCH_FILE}")

    mode_cfg = MODES[case.mode]
    humble = "/opt/ros/humble/setup.bash"
    ws_setup = str(WS / "install/setup.bash")
    mode = mode_cfg.get("stabilization_mode", "")
    mode_arg = f" stabilization_mode:={mode}" if mode else ""
    bash_cmd = f"""
source {humble}
source {ws_setup}
ros2 launch {LAUNCH_FILE} \\
  use_ik_joint_control:={_bool_str(mode_cfg["use_ik_joint_control"])} \\
  kinematic_stabilization:={_bool_str(mode_cfg["kinematic_stabilization"])}{mode_arg} \\
  disturbance_radius:={case.radius:.6f} \\
  disturbance_orient_amp:={case.orient_amp:.6f} \\
  disturbance_time_constant:={case.time_constant:.6f} \\
  disturbance_amplitude_scale:={case.amplitude_scale:.6f}
"""
    return subprocess.Popen(
        ["bash", "-lc", bash_cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=os.environ.copy(),
        preexec_fn=os.setsid,
    )


def stop_proc(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    kill_stale()


def _stamp_nsec(msg: JointState) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _extract_arm_q(msg: JointState) -> Optional[np.ndarray]:
    q = np.zeros(6)
    got = 0
    for i, name in enumerate(ARM_JOINTS):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if idx < len(msg.position):
            q[i] = msg.position[idx]
            got += 1
    return q if got == len(ARM_JOINTS) else None


def _extract_mount(msg: JointState) -> Optional[tuple[np.ndarray, np.ndarray]]:
    pos = np.zeros(3)
    vel = np.zeros(6)
    got = 0
    for i, name in enumerate(MOUNT_JOINTS):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if i < 3 and idx < len(msg.position):
            pos[i] = msg.position[idx]
        if idx < len(msg.velocity):
            vel[i] = msg.velocity[idx]
        got += 1
    return (pos, vel) if got == len(MOUNT_JOINTS) else None


def _extract_ref(msg: JointState) -> Optional[np.ndarray]:
    q_cmd = np.zeros(6)
    got = 0
    for i, name in enumerate(ARM_JOINTS):
        if name not in msg.name:
            continue
        idx = msg.name.index(name)
        if idx < len(msg.velocity):
            q_cmd[i] = msg.velocity[idx]
        elif idx < len(msg.position):
            q_cmd[i] = msg.position[idx]
        got += 1
    return q_cmd if got == len(ARM_JOINTS) else None


class LimitTestRecorder(Node):
    def __init__(self, warmup_sec: float, record_sec: float):
        super().__init__("limit_test_recorder")
        self.warmup_sec = warmup_sec
        self.record_sec = record_sec
        self.t0 = time.time()
        self.t_record0_nsec: Optional[int] = None
        self._last_sample_t = -1.0
        self.target_locked = False
        self.samples: list[Sample] = []

        self._err_by_stamp: dict[int, tuple[float, float, float, float, float]] = {}
        self._js_by_stamp: dict[int, np.ndarray] = {}
        self._mount_by_stamp: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._ref_by_stamp: dict[int, np.ndarray] = {}

        self.create_subscription(MarkerArray, "/stabilization_markers", self._marker_cb, 10)
        self.create_subscription(Float64MultiArray, "/stabilization_error", self._err_cb, 50)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self.create_subscription(JointState, "/stabilization_reference", self._ref_cb, 50)

    def _marker_cb(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            if marker.id == 1:
                self.target_locked = True
                break

    def _err_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            return
        stamp = self.get_clock().now().nanoseconds
        self._err_by_stamp[stamp] = tuple(float(x) for x in msg.data[:5])
        self._trim_dict(self._err_by_stamp)

    def _joint_cb(self, msg: JointState) -> None:
        q_act = _extract_arm_q(msg)
        mount = _extract_mount(msg)
        if q_act is None or mount is None:
            return
        stamp = _stamp_nsec(msg)
        self._js_by_stamp[stamp] = q_act
        self._mount_by_stamp[stamp] = mount
        self._trim_dict(self._js_by_stamp)
        self._trim_dict(self._mount_by_stamp)
        self._try_sample(stamp)

    def _ref_cb(self, msg: JointState) -> None:
        q_cmd = _extract_ref(msg)
        if q_cmd is None:
            return
        stamp = _stamp_nsec(msg)
        self._ref_by_stamp[stamp] = q_cmd
        self._trim_dict(self._ref_by_stamp)
        self._try_sample(stamp)

    @staticmethod
    def _trim_dict(d: dict, keep: int = 1000) -> None:
        if len(d) > keep:
            for key in sorted(d)[:-keep]:
                d.pop(key, None)

    def _elapsed(self) -> float:
        return time.time() - self.t0

    def _nearest_err(self, stamp: int) -> Optional[tuple[float, float, float, float, float]]:
        if not self._err_by_stamp:
            return None
        key = min(self._err_by_stamp, key=lambda k: abs(k - stamp))
        if abs(key - stamp) > 50_000_000:
            return None
        return self._err_by_stamp[key]

    def _try_sample(self, stamp: int) -> None:
        if stamp not in self._js_by_stamp or stamp not in self._mount_by_stamp:
            return
        if self._elapsed() < self.warmup_sec:
            return
        if self.t_record0_nsec is None:
            self.t_record0_nsec = stamp
        t = (stamp - self.t_record0_nsec) / 1e9
        if t >= self.record_sec:
            return
        if t - self._last_sample_t < 0.018:
            return

        err = self._nearest_err(stamp)
        if err is None:
            return

        q_cmd = self._ref_by_stamp.get(stamp)
        if q_cmd is None and self._ref_by_stamp:
            ref_key = min(self._ref_by_stamp, key=lambda k: abs(k - stamp))
            if abs(ref_key - stamp) < 50_000_000:
                q_cmd = self._ref_by_stamp[ref_key]
        if q_cmd is None:
            q_cmd = np.zeros(6)

        mount_pos, mount_vel = self._mount_by_stamp[stamp]
        self._last_sample_t = t
        self.samples.append(
            Sample(
                t=t,
                pos_err_m=err[0],
                orient_err_rad=err[1],
                base_pos_err_m=err[2],
                base_orient_err_rad=err[3],
                ik_solve_us=err[4],
                mount_pos=mount_pos.copy(),
                mount_vel=mount_vel.copy(),
                q_act=self._js_by_stamp[stamp].copy(),
                q_cmd=q_cmd.copy(),
            )
        )


def record_case(
    case: DisturbanceCase,
    warmup_sec: float = WARMUP_SEC,
    record_sec: float = RECORD_SEC,
    startup_wait: float = 5.0,
) -> RunRecord:
    kill_stale()
    proc = launch_case(case)
    time.sleep(startup_wait)

    if not rclpy.ok():
        rclpy.init()
    node = LimitTestRecorder(warmup_sec, record_sec)
    deadline = time.time() + warmup_sec + record_sec + 5.0
    try:
        while time.time() < deadline and rclpy.ok():
            if proc.poll() is not None:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        record = RunRecord(
            case=case,
            samples=node.samples,
            target_locked=node.target_locked,
            launch_returncode=proc.returncode,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    stop_proc(proc)
    if len(record.samples) < 50:
        raise RuntimeError(
            f"Insufficient samples for {case.run_id}: {len(record.samples)} "
            f"(target_locked={record.target_locked})"
        )
    return record


def measure_baseline_pos_rms() -> float:
    case = DisturbanceCase(
        study="baseline",
        mode="A",
        radius=NOMINAL_RADIUS,
        orient_amp=NOMINAL_ORIENT_AMP,
        time_constant=NOMINAL_TIME_CONSTANT,
        scale=1.0,
    )
    record = record_case(case, startup_wait=5.0)
    metrics = compute_metrics(record)
    return float(metrics["position_mm"]["rms"])
