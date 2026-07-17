#!/usr/bin/env python3
"""真机自稳失效分析数据录制（ee-stabilization-vicon）。

同时采集：
  - 控制误差 / 规划耗时          /stabilization_error
  - 锁定目标与末端可视化         /stabilization_markers
  - 关节反馈                     /joint_states
  - 关节指令（含力矩前馈）       /student/joint_command
  - 电机反馈（若有）             /student/joint_feedback
  - 飞机绝对位姿                 /vrpn/<rigid>/pose
  - 相对扰动 Δ                   /vicon_relative/delta
  - 扰动数组                     /mount_disturbance/pose
  - TF world→base_link
  - （可选）ros2 bag 原始话题

输出目录:
  <out>/
    run_meta.json
    summary.txt
    aligned.csv              # 按时钟对齐的主表，优先用这个分析
    streams/*.csv            # 各话题原始流
    events.csv               # 手动标记（故障起点等）
    bag/                     # --bag 时生成

用法（另开终端，先跑 ./run_hw.sh C）:
  source /opt/ros/humble/setup.bash
  source ~/zihan_ws/arm/windylab_ws/install/setup.bash
  python3 scripts/record_failure_analysis.py --duration 60 --note "modeC_shake"

  # Ctrl+C 提前结束也会落盘；运行中按 Enter 可标记 failure 事件。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

try:
    from dummy_interface.msg import MotorState
except ImportError:  # pragma: no cover
    MotorState = None  # type: ignore

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
ARM6 = JOINT_NAMES[:6]

# /stabilization_error data[]
ERR_FIELDS = (
    "world_pos_err_m",
    "world_orient_err_rad",
    "task_pos_err_m",
    "task_orient_err_rad",
    "ik_solve_us",
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _stamp_sec_header(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    # ZYX yaw-pitch-roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _write_csv(path: Path, rows: list[dict]) -> None:
    """Write CSV atomically (temp file + replace) so autosave cannot truncate."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    # stable header: union of keys in first-seen order
    keys: list[str] = []
    seen = set()
    for row in rows:
        for k in row:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)


@dataclass
class Stream:
    name: str
    rows: list[dict] = field(default_factory=list)

    def add(self, row: dict) -> None:
        self.rows.append(row)


class FailureAnalysisRecorder(Node):
    def __init__(
        self,
        pose_topic: str,
        world_frame: str,
        base_frame: str,
        sample_hz: float,
        ee_link: str,
    ) -> None:
        super().__init__("failure_analysis_recorder")
        self.pose_topic = pose_topic
        self.world_frame = world_frame
        self.base_frame = base_frame
        self.ee_link = ee_link
        self.sample_dt = 1.0 / max(sample_hz, 1.0)

        self.wall_t0 = time.time()
        self.target_locked = False
        self.events: list[dict] = []

        self.err = Stream("stabilization_error")
        self.markers = Stream("markers")
        self.joint_states = Stream("joint_states")
        self.joint_cmd = Stream("joint_command")
        self.joint_fb = Stream("joint_feedback")
        self.vrpn = Stream("vrpn_pose")
        self.delta = Stream("vicon_delta")
        self.mount = Stream("mount_disturbance")
        self.tf_base = Stream("tf_world_base")
        self.tf_ee = Stream("tf_world_ee")
        self.aligned: list[dict] = []

        # latest caches for alignment
        self._latest: dict[str, dict] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Float64MultiArray, "/stabilization_error", self._on_err, reliable_qos
        )
        self.create_subscription(
            MarkerArray, "/stabilization_markers", self._on_markers, 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, sensor_qos)
        self.create_subscription(
            JointState, "/student/joint_command", self._on_cmd, reliable_qos
        )
        self.create_subscription(
            PoseStamped, pose_topic, self._on_vrpn, sensor_qos
        )
        self.create_subscription(
            PoseStamped, "/vicon_relative/delta", self._on_delta, reliable_qos
        )
        self.create_subscription(
            Float64MultiArray, "/mount_disturbance/pose", self._on_mount, reliable_qos
        )
        if MotorState is not None:
            self.create_subscription(
                MotorState, "/student/joint_feedback", self._on_fb, sensor_qos
            )
        else:
            self.get_logger().warn(
                "dummy_interface not importable; skip /student/joint_feedback"
            )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(self.sample_dt, self._on_sample_timer)

        self.get_logger().info(
            f"Recording failure-analysis streams @ {sample_hz:.0f} Hz | "
            f"pose={pose_topic} frames={world_frame}->{base_frame}/{ee_link}"
        )

    def rel_t(self) -> float:
        return time.time() - self.wall_t0

    def mark_event(self, label: str, note: str = "") -> None:
        row = {"t_sec": round(self.rel_t(), 6), "label": label, "note": note}
        self.events.append(row)
        self.get_logger().info(f"event @ {row['t_sec']:.3f}s: {label} {note}".rstrip())

    # ---- callbacks ----

    def _on_err(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            return
        row = {"t_sec": round(self.rel_t(), 6)}
        for i, name in enumerate(ERR_FIELDS):
            row[name] = float(msg.data[i])
        self.err.add(row)
        self._latest["err"] = row

    def _on_markers(self, msg: MarkerArray) -> None:
        t = round(self.rel_t(), 6)
        row: dict[str, Any] = {"t_sec": t}
        for m in msg.markers:
            if m.id == 1:
                self.target_locked = True
                row["target_x"] = float(m.pose.position.x)
                row["target_y"] = float(m.pose.position.y)
                row["target_z"] = float(m.pose.position.z)
            elif m.id == 2:
                row["ee_x"] = float(m.pose.position.x)
                row["ee_y"] = float(m.pose.position.y)
                row["ee_z"] = float(m.pose.position.z)
            elif m.id == 3:
                row["mount_marker_x"] = float(m.pose.position.x)
                row["mount_marker_y"] = float(m.pose.position.y)
                row["mount_marker_z"] = float(m.pose.position.z)
        if len(row) > 1:
            self.markers.add(row)
            self._latest["markers"] = row

    def _joint_row(self, msg: JointState, include_effort: bool) -> dict:
        row: dict[str, Any] = {
            "t_sec": round(self.rel_t(), 6),
            "stamp_sec": _stamp_sec_header(msg.header.stamp),
        }
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for jn in JOINT_NAMES:
            idx = name_to_idx.get(jn)
            if idx is None:
                row[f"{jn}_pos"] = float("nan")
                row[f"{jn}_vel"] = float("nan")
                if include_effort:
                    row[f"{jn}_eff"] = float("nan")
                continue
            row[f"{jn}_pos"] = (
                float(msg.position[idx]) if idx < len(msg.position) else float("nan")
            )
            row[f"{jn}_vel"] = (
                float(msg.velocity[idx]) if idx < len(msg.velocity) else float("nan")
            )
            if include_effort:
                row[f"{jn}_eff"] = (
                    float(msg.effort[idx]) if idx < len(msg.effort) else float("nan")
                )
        return row

    def _on_js(self, msg: JointState) -> None:
        row = self._joint_row(msg, include_effort=True)
        self.joint_states.add(row)
        self._latest["js"] = row

    def _on_cmd(self, msg: JointState) -> None:
        # HW: position=q_cmd, velocity=dq, effort=tau_ff
        row = self._joint_row(msg, include_effort=True)
        # clearer aliases for analysis
        for jn in JOINT_NAMES:
            row[f"{jn}_q_cmd"] = row.get(f"{jn}_pos", float("nan"))
            row[f"{jn}_dq_cmd"] = row.get(f"{jn}_vel", float("nan"))
            row[f"{jn}_tau_ff"] = row.get(f"{jn}_eff", float("nan"))
        self.joint_cmd.add(row)
        self._latest["cmd"] = row

    def _on_fb(self, msg) -> None:  # MotorState
        row: dict[str, Any] = {
            "t_sec": round(self.rel_t(), 6),
            "stamp_sec": _stamp_sec_header(msg.header.stamp),
        }
        n = max(
            len(msg.position),
            len(msg.velocity),
            len(msg.current),
            len(msg.voltage),
            len(msg.temperature),
            7,
        )
        for i in range(min(n, 7)):
            jn = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"j{i+1}"
            row[f"{jn}_fb_pos"] = float(msg.position[i]) if i < len(msg.position) else float("nan")
            row[f"{jn}_fb_vel"] = float(msg.velocity[i]) if i < len(msg.velocity) else float("nan")
            row[f"{jn}_current"] = float(msg.current[i]) if i < len(msg.current) else float("nan")
            row[f"{jn}_voltage"] = float(msg.voltage[i]) if i < len(msg.voltage) else float("nan")
            row[f"{jn}_temp"] = (
                float(msg.temperature[i]) if i < len(msg.temperature) else float("nan")
            )
        self.joint_fb.add(row)
        self._latest["fb"] = row

    def _pose_row(self, msg: PoseStamped, prefix: str) -> dict:
        p = msg.pose.position
        q = msg.pose.orientation
        r, pitch, y = _quat_to_rpy(q.x, q.y, q.z, q.w)
        return {
            "t_sec": round(self.rel_t(), 6),
            "stamp_sec": _stamp_sec_header(msg.header.stamp),
            f"{prefix}_x": float(p.x),
            f"{prefix}_y": float(p.y),
            f"{prefix}_z": float(p.z),
            f"{prefix}_qx": float(q.x),
            f"{prefix}_qy": float(q.y),
            f"{prefix}_qz": float(q.z),
            f"{prefix}_qw": float(q.w),
            f"{prefix}_roll": r,
            f"{prefix}_pitch": pitch,
            f"{prefix}_yaw": y,
        }

    def _on_vrpn(self, msg: PoseStamped) -> None:
        row = self._pose_row(msg, "plane")
        self.vrpn.add(row)
        self._latest["vrpn"] = row

    def _on_delta(self, msg: PoseStamped) -> None:
        row = self._pose_row(msg, "delta")
        self.delta.add(row)
        self._latest["delta"] = row

    def _on_mount(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            return
        row = {
            "t_sec": round(self.rel_t(), 6),
            "mount_x": float(msg.data[0]),
            "mount_y": float(msg.data[1]),
            "mount_z": float(msg.data[2]),
            "mount_roll": float(msg.data[3]),
            "mount_pitch": float(msg.data[4]),
            "mount_yaw": float(msg.data[5]),
        }
        self.mount.add(row)
        self._latest["mount"] = row

    def _lookup_tf(self, parent: str, child: str) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time()
            )
        except TransformException:
            return None

    def _tf_row(self, parent: str, child: str, prefix: str) -> Optional[dict]:
        tf = self._lookup_tf(parent, child)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        r, pitch, y = _quat_to_rpy(q.x, q.y, q.z, q.w)
        return {
            "t_sec": round(self.rel_t(), 6),
            "stamp_sec": _stamp_sec_header(tf.header.stamp),
            f"{prefix}_x": float(t.x),
            f"{prefix}_y": float(t.y),
            f"{prefix}_z": float(t.z),
            f"{prefix}_qx": float(q.x),
            f"{prefix}_qy": float(q.y),
            f"{prefix}_qz": float(q.z),
            f"{prefix}_qw": float(q.w),
            f"{prefix}_roll": r,
            f"{prefix}_pitch": pitch,
            f"{prefix}_yaw": y,
            f"{prefix}_ok": 1,
        }

    def _on_sample_timer(self) -> None:
        t = round(self.rel_t(), 6)
        row: dict[str, Any] = {
            "t_sec": t,
            "target_locked": int(self.target_locked),
        }

        # TF snapshots
        base = self._tf_row(self.world_frame, self.base_frame, "tf_base")
        if base is not None:
            self.tf_base.add(base)
            self._latest["tf_base"] = base
            row.update({k: v for k, v in base.items() if k != "t_sec"})
        else:
            row["tf_base_ok"] = 0

        ee = self._tf_row(self.world_frame, self.ee_link, "tf_ee")
        if ee is not None:
            self.tf_ee.add(ee)
            self._latest["tf_ee"] = ee
            row.update({k: v for k, v in ee.items() if k != "t_sec"})
        else:
            row["tf_ee_ok"] = 0

        for key in ("err", "markers", "js", "cmd", "fb", "vrpn", "delta", "mount"):
            latest = self._latest.get(key)
            if latest is None:
                continue
            # drop mismatched timestamps older than 0.2 s
            if abs(latest["t_sec"] - t) > 0.2:
                continue
            for k, v in latest.items():
                if k == "t_sec":
                    continue
                # Keep measured joint_* from js; cmd uses joint*_q_cmd / dq_cmd / tau_ff.
                if key == "cmd" and (
                    k.endswith("_pos") or k.endswith("_vel") or k.endswith("_eff")
                ) and not (
                    k.endswith("_q_cmd") or k.endswith("_dq_cmd") or k.endswith("_tau_ff")
                ):
                    continue
                row[k] = v

        # derived joint tracking error (cmd - act) for first 6 DOF
        for jn in ARM6:
            qp = row.get(f"{jn}_q_cmd")
            qa = row.get(f"{jn}_pos")
            if qp is not None and qa is not None and not (
                isinstance(qp, float) and math.isnan(qp)
            ) and not (isinstance(qa, float) and math.isnan(qa)):
                row[f"{jn}_track_err"] = float(qp) - float(qa)

        # mount magnitude from delta or mount array
        if "delta_x" in row:
            row["delta_pos_norm"] = math.sqrt(
                row["delta_x"] ** 2 + row["delta_y"] ** 2 + row["delta_z"] ** 2
            )
        elif "mount_x" in row:
            row["delta_pos_norm"] = math.sqrt(
                row["mount_x"] ** 2 + row["mount_y"] ** 2 + row["mount_z"] ** 2
            )

        self.aligned.append(row)


def _stdin_event_loop(node: FailureAnalysisRecorder, stop_evt: threading.Event) -> None:
    """Press Enter to mark a 'failure' event (non-blocking when TTY)."""
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_evt.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r", "f", "F"):
                node.mark_event("failure_mark", "user key")
            elif ch in ("n", "N"):
                node.mark_event("note", "user key")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _start_bag(out_dir: Path, pose_topic: str) -> Optional[subprocess.Popen]:
    bag_dir = out_dir / "bag"
    topics = [
        "/stabilization_error",
        "/stabilization_markers",
        "/joint_states",
        "/student/joint_command",
        "/student/joint_feedback",
        pose_topic,
        "/vicon_relative/delta",
        "/mount_disturbance/pose",
        "/tf",
        "/tf_static",
        "/robot_description",
    ]
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(bag_dir),
        "--compression-mode",
        "file",
        "--compression-format",
        "zstd",
        *topics,
    ]
    try:
        return subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
    except FileNotFoundError:
        return None


def _stop_bag(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), 2)  # SIGINT for clean bag close
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except (ProcessLookupError, PermissionError):
            pass


def _summarize(aligned: list[dict], events: list[dict]) -> str:
    lines = []
    lines.append("=== failure analysis summary ===")
    if not aligned:
        lines.append("no aligned samples")
        return "\n".join(lines) + "\n"

    def series(key: str) -> list[float]:
        out = []
        for r in aligned:
            v = r.get(key)
            if v is None:
                continue
            try:
                fv = float(v)
            except (TypeError, ValueError):
                continue
            if math.isnan(fv):
                continue
            out.append(fv)
        return out

    def stats(name: str, key: str, scale: float = 1.0, unit: str = "") -> None:
        xs = series(key)
        if not xs:
            lines.append(f"{name}: (no data)")
            return
        import statistics as st

        rms = math.sqrt(sum(x * x for x in xs) / len(xs))
        p95 = sorted(xs)[max(0, int(0.95 * (len(xs) - 1)))]
        lines.append(
            f"{name}: n={len(xs)}  mean={st.mean(xs)*scale:.4g}{unit}  "
            f"rms={rms*scale:.4g}{unit}  max={max(xs)*scale:.4g}{unit}  "
            f"p95={p95*scale:.4g}{unit}"
        )

    lines.append(f"duration_sec: {aligned[-1]['t_sec']:.2f}")
    lines.append(f"aligned_samples: {len(aligned)}")
    lines.append(f"target_locked_end: {bool(aligned[-1].get('target_locked', 0))}")
    stats("world_pos_err", "world_pos_err_m", 1000.0, " mm")
    stats("world_orient_err", "world_orient_err_rad", 1.0, " rad")
    stats("task_pos_err", "task_pos_err_m", 1000.0, " mm")
    stats("ik_solve", "ik_solve_us", 1.0, " us")
    stats("delta_pos_norm", "delta_pos_norm", 1000.0, " mm")
    tf_ok = sum(1 for r in aligned if r.get("tf_base_ok", 0) == 1 or "tf_base_x" in r)
    lines.append(f"tf_base_present_samples: {tf_ok}/{len(aligned)}")
    cmd_n = sum(1 for r in aligned if any(f"{j}_q_cmd" in r for j in ARM6))
    lines.append(f"samples_with_joint_cmd: {cmd_n}/{len(aligned)}")

    # divergence check: last 2s vs first 2s pos err
    early = [r["world_pos_err_m"] for r in aligned if r["t_sec"] <= 2.0 and "world_pos_err_m" in r]
    late_t0 = max(0.0, aligned[-1]["t_sec"] - 2.0)
    late = [
        r["world_pos_err_m"]
        for r in aligned
        if r["t_sec"] >= late_t0 and "world_pos_err_m" in r
    ]
    if early and late:
        e_rms = math.sqrt(sum(x * x for x in early) / len(early))
        l_rms = math.sqrt(sum(x * x for x in late) / len(late))
        lines.append(
            f"pos_err_rms early2s={e_rms*1000:.3f} mm  late2s={l_rms*1000:.3f} mm  "
            f"ratio={l_rms / max(e_rms, 1e-9):.2f}"
        )
        if l_rms > max(0.02, 3.0 * e_rms):
            lines.append("FLAG: possible divergence (late pos error >> early)")

    lines.append(f"manual_events: {len(events)}")
    for ev in events:
        lines.append(f"  - t={ev['t_sec']:.3f}s  {ev['label']}  {ev.get('note','')}")
    lines.append("")
    lines.append("How to diagnose:")
    lines.append("  1) If delta_pos_norm grows but err stays flat → compensation OK")
    lines.append("  2) If delta grows AND world_pos_err grows → stabilization failure")
    lines.append("  3) If joint*_track_err large → tracking/actuator issue")
    lines.append("  4) If tf_base_ok=0 often → Vicon/bridge/TF dropout")
    lines.append("  5) If currents spike / nan → motor / serial issue")
    return "\n".join(lines) + "\n"


def save_outputs(
    out_dir: Path,
    node: FailureAnalysisRecorder,
    args: argparse.Namespace,
    bag_ok: bool,
    quiet: bool = False,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    streams = out_dir / "streams"
    _write_csv(streams / "stabilization_error.csv", node.err.rows)
    _write_csv(streams / "markers.csv", node.markers.rows)
    _write_csv(streams / "joint_states.csv", node.joint_states.rows)
    _write_csv(streams / "joint_command.csv", node.joint_cmd.rows)
    _write_csv(streams / "joint_feedback.csv", node.joint_fb.rows)
    _write_csv(streams / "vrpn_pose.csv", node.vrpn.rows)
    _write_csv(streams / "vicon_delta.csv", node.delta.rows)
    _write_csv(streams / "mount_disturbance.csv", node.mount.rows)
    _write_csv(streams / "tf_world_base.csv", node.tf_base.rows)
    _write_csv(streams / "tf_world_ee.csv", node.tf_ee.rows)
    _write_csv(out_dir / "aligned.csv", node.aligned)
    _write_csv(out_dir / "events.csv", node.events)

    summary = _summarize(node.aligned, node.events)
    (out_dir / "summary.txt").write_text(summary, encoding="utf-8")

    meta = {
        "created_at": _now_iso(),
        "note": args.note,
        "mode_hint": args.mode,
        "duration_requested_sec": args.duration,
        "duration_actual_sec": node.aligned[-1]["t_sec"] if node.aligned else 0.0,
        "sample_hz": args.hz,
        "pose_topic": args.pose_topic,
        "world_frame": args.world_frame,
        "base_frame": args.base_frame,
        "ee_link": args.ee_link,
        "target_locked": node.target_locked,
        "bag_recorded": bag_ok,
        "counts": {
            "aligned": len(node.aligned),
            "stabilization_error": len(node.err.rows),
            "joint_states": len(node.joint_states.rows),
            "joint_command": len(node.joint_cmd.rows),
            "joint_feedback": len(node.joint_fb.rows),
            "vrpn_pose": len(node.vrpn.rows),
            "vicon_delta": len(node.delta.rows),
            "mount_disturbance": len(node.mount.rows),
            "markers": len(node.markers.rows),
            "tf_world_base": len(node.tf_base.rows),
            "tf_world_ee": len(node.tf_ee.rows),
            "events": len(node.events),
        },
        "topics": [
            "/stabilization_error",
            "/stabilization_markers",
            "/joint_states",
            "/student/joint_command",
            "/student/joint_feedback",
            args.pose_topic,
            "/vicon_relative/delta",
            "/mount_disturbance/pose",
            "/tf (world→base_link, world→ee)",
        ],
        "error_fields": list(ERR_FIELDS),
        "analysis_tips": [
            "Prefer aligned.csv for timeline plots",
            "Cross-check delta_pos_norm vs world_pos_err_m for failure onset",
            "joint*_track_err separates control vs actuation",
            "tf_base_ok / vrpn streams diagnose perception dropouts",
            "events.csv marks user-noted failure moments (Enter/f)",
        ],
    }
    (out_dir / "run_meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    if not quiet:
        print(summary)
        print(f"Saved to: {out_dir}")


def parse_args() -> argparse.Namespace:
    project = Path(__file__).resolve().parents[1]
    default_root = project / "data" / "runs"
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Record seconds; 0 = continuous until Ctrl+C",
    )
    p.add_argument("--hz", type=float, default=50.0, help="Aligned sample rate (default 50)")
    p.add_argument("--note", type=str, default="", help="Free-text note stored in meta")
    p.add_argument("--mode", type=str, default="", help="Mode hint A/B/C/D for meta only")
    p.add_argument("--out", type=str, default="", help="Output directory (default data/runs/<stamp>)")
    p.add_argument("--out-root", type=str, default=str(default_root))
    p.add_argument("--pose-topic", type=str, default=os.environ.get("POSE_TOPIC", "/vrpn/pregme/pose"))
    p.add_argument("--world-frame", type=str, default="world")
    p.add_argument("--base-frame", type=str, default="base_link")
    p.add_argument("--ee-link", type=str, default="link6")
    p.add_argument("--bag", action="store_true", help="Also record compressed ros2 bag")
    p.add_argument("--no-stdin-events", action="store_true", help="Disable Enter/f event marks")
    p.add_argument(
        "--autosave-sec",
        type=float,
        default=30.0,
        help="Periodic flush to disk (0 disables; default 30)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = args.mode.strip() or "run"
    note_slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.note)[:40]
    name = f"{stamp}_{label}" + (f"_{note_slug}" if note_slug else "")
    out_dir = Path(args.out) if args.out else Path(args.out_root) / name
    out_dir.mkdir(parents=True, exist_ok=True)

    if not rclpy.ok():
        rclpy.init()

    node = FailureAnalysisRecorder(
        pose_topic=args.pose_topic,
        world_frame=args.world_frame,
        base_frame=args.base_frame,
        sample_hz=args.hz,
        ee_link=args.ee_link,
    )
    node.mark_event("start", args.note)

    bag_proc = _start_bag(out_dir, args.pose_topic) if args.bag else None
    if args.bag and bag_proc is None:
        node.get_logger().warn("Failed to start ros2 bag; continuing with CSV only")

    stop_evt = threading.Event()
    kb_thread = None
    if not args.no_stdin_events and sys.stdin.isatty():
        print("Recording… Press Enter/f to mark failure event; Ctrl+C to stop & save.", flush=True)
        kb_thread = threading.Thread(
            target=_stdin_event_loop, args=(node, stop_evt), daemon=True
        )
        kb_thread.start()
    else:
        forever = args.duration <= 0
        print(
            f"Recording… {'continuous until stop' if forever else f'{args.duration:.0f}s'}; "
            f"Ctrl+C to stop & save. out={out_dir}",
            flush=True,
        )

    continuous = args.duration <= 0
    deadline = None if continuous else time.time() + max(0.1, args.duration)
    next_autosave = time.time() + max(0.0, args.autosave_sec)
    try:
        while rclpy.ok():
            if deadline is not None and time.time() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.autosave_sec > 0 and time.time() >= next_autosave:
                save_outputs(out_dir, node, args, bag_ok=False, quiet=True)
                next_autosave = time.time() + args.autosave_sec
                node.get_logger().info(
                    f"autosave n_aligned={len(node.aligned)} -> {out_dir}"
                )
    except KeyboardInterrupt:
        print("\nInterrupted — saving…", flush=True)
        node.mark_event("interrupt", "KeyboardInterrupt")
    finally:
        stop_evt.set()
        node.mark_event("stop", "")
        _stop_bag(bag_proc)
        bag_ok = bool(args.bag and bag_proc is not None and (out_dir / "bag").exists())
        try:
            save_outputs(out_dir, node, args, bag_ok=bag_ok, quiet=False)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
