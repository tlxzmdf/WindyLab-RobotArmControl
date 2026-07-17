#!/usr/bin/env python3
"""Hardware MIT trajectory test: cosine / line / circle.

Aligned with arm-platform student demos:
  - move_arm_line_demo.py  (A↔B cosine) → task ``cosine`` / ``line``
  - move_arm_ik_demo.py    (YZ circle r=0.08) → task ``circle``
  - PinocchioIK position IK, velocity=0 on /student/joint_command

Fixes vs first HW run:
  - Do NOT block the control loop with matplotlib (caused Command timeout)
  - Abort if joint feedback stays frozen (last run: q_meas≡0)
  - Use absolute demo waypoints by default
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Import PinocchioIK the same way demos do (arm-platform/demo on sys.path)
_PROJECT = Path(__file__).resolve().parents[1]
_ARM_ROOT = _PROJECT.parents[1]  # .../arm
_DEMO_DIR = _ARM_ROOT / "windylab_ws" / "src" / "arm-platform" / "demo"
if not _DEMO_DIR.is_dir():
    raise RuntimeError(f"arm-platform demo dir not found: {_DEMO_DIR}")
sys.path.insert(0, str(_DEMO_DIR))
from pinocchio_ik import PinocchioIK  # noqa: E402

from hw_mit_traj_lib import (  # noqa: E402
    DEMO_CIRCLE_CENTER,
    DEMO_CIRCLE_RADIUS,
    DEMO_POINT_A,
    DEMO_POINT_B,
    JOINT_NAMES,
    PROJECT_ROOT,
    ArmKinematics,
    PoseSample,
    build_task_trajectory,
    pad7,
    resolve_urdf,
    write_csv,
    write_plots_and_metrics,
    _rpy,
)

DEFAULT_Q_HOME = [0.0, 0.35, -0.55, 0.0, 0.45, 0.0, 0.0]


def extract_q_dq(
    msg: JointState, n: int
) -> Optional[tuple[list[float], list[float]]]:
    name_to_idx = {name: i for i, name in enumerate(msg.name)}
    q: list[float] = []
    dq: list[float] = []
    for i in range(n):
        name = JOINT_NAMES[i]
        if name not in name_to_idx:
            return None
        idx = name_to_idx[name]
        if idx >= len(msg.position):
            return None
        q.append(float(msg.position[idx]))
        if msg.velocity and idx < len(msg.velocity):
            dq.append(float(msg.velocity[idx]))
        else:
            dq.append(0.0)
    return q, dq


class MitTrajRunner(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hw_mit_traj_test")
        self.args = args
        self.kin = ArmKinematics(resolve_urdf(args.urdf), ee_frame=args.ee_frame)
        self.ik = PinocchioIK(urdf_path=str(self.kin.urdf_path), ee_frame=self.kin.ee_name)
        self.nq = self.kin.nq
        self.n_pub = 7
        self.dt = 1.0 / max(args.rate, 1.0)
        self.q_meas: Optional[np.ndarray] = None
        self.dq_meas: Optional[np.ndarray] = None
        self.q_cmd = np.zeros(self.nq)
        self.dq_cmd = np.zeros(self.nq)
        self.q_last_ik: Optional[np.ndarray] = None
        self.samples: list[PoseSample] = []
        self.pending_plot_dirs: list[Path] = []
        self.phase = "wait_js"
        self.phase_t0: Optional[float] = None
        self.traj: list = []
        self.traj_idx = 0
        self.task_name = ""
        self.p0 = np.zeros(3)
        self.R0 = np.eye(3)
        self.q_home = np.array(args.q_home[: self.nq], dtype=float)
        if self.q_home.size < self.nq:
            self.q_home = np.pad(self.q_home, (0, self.nq - self.q_home.size))
        self.home_start: Optional[np.ndarray] = None
        self.approach_start: Optional[np.ndarray] = None
        self.approach_goal: Optional[np.ndarray] = None
        self.zero_start: Optional[np.ndarray] = None
        self.q_zero = np.zeros(self.nq)
        self._pending_after_zero: Optional[str] = None  # next task name or "" if finish
        self.done = False
        self.failed = False
        self.run_dir: Optional[Path] = None
        self._js_count = 0
        self._js_moved = False
        self.run_dirs: list[Path] = []

        self.pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_js, 50
        )
        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f"URDF={self.kin.urdf_path} ee={self.kin.ee_name} nq={self.nq} "
            f"rate={args.rate:.1f}Hz task={args.task} "
            f"demo_waypoints={args.demo_waypoints}"
        )

    def _on_js(self, msg: JointState) -> None:
        got = extract_q_dq(msg, max(self.nq, 6))
        if got is None:
            return
        q, dq = got
        q_arr = np.array(q[: self.nq], dtype=float)
        dq_arr = np.array(dq[: self.nq], dtype=float)
        if q_arr.size < self.nq:
            q_arr = np.pad(q_arr, (0, self.nq - q_arr.size))
            dq_arr = np.pad(dq_arr, (0, self.nq - dq_arr.size))
        if self.q_meas is not None and np.linalg.norm(q_arr - self.q_meas) > 1e-4:
            self._js_moved = True
        self.q_meas = q_arr
        self.dq_meas = dq_arr
        self._js_count += 1

    def _publish(self, q: np.ndarray, dq: Optional[np.ndarray] = None) -> None:
        """Publish MIT position command. velocity defaults to 0 (student demos)."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        n = self.n_pub
        qq = pad7(q)
        if dq is None or self.args.zero_velocity:
            dd = np.zeros(7)
        else:
            dd = pad7(dq)
        msg.name = JOINT_NAMES[:n]
        msg.position = [float(qq[i]) for i in range(n)]
        msg.velocity = [float(dd[i]) for i in range(n)]
        msg.effort = [0.0] * n
        self.pub.publish(msg)

    def _hold_current(self) -> None:
        if self.q_meas is None:
            return
        # Hold measured pose (safe); keep streaming to avoid command timeout
        self._publish(self.q_meas, np.zeros(self.nq))

    def _start_phase(self, name: str) -> None:
        self.phase = name
        self.phase_t0 = time.time()
        self.get_logger().info(f"phase -> {name}")

    def _elapsed(self) -> float:
        assert self.phase_t0 is not None
        return time.time() - self.phase_t0

    def _ik_to(self, p: np.ndarray) -> Optional[np.ndarray]:
        q_init = self.q_last_ik if self.q_last_ik is not None else self.q_meas
        q, ok = self.ik.solve(np.asarray(p, dtype=float), q_init=q_init)
        if not ok:
            self.get_logger().warn(
                f"IK not converged for p={np.round(p, 4).tolist()}",
                throttle_duration_sec=1.0,
            )
            return None
        self.q_last_ik = q.copy()
        return q

    def _fail(self, msg: str) -> None:
        self.get_logger().error(msg)
        self.failed = True
        self.done = True

    def _begin_approach(self, task: str) -> None:
        assert self.q_meas is not None
        self.task_name = task
        if self.args.demo_waypoints:
            if task == "circle":
                p_start = DEMO_CIRCLE_CENTER + np.array(
                    [0.0, DEMO_CIRCLE_RADIUS, 0.0]
                )
            else:
                p_start = DEMO_POINT_A.copy()
        else:
            p_start, _ = self.kin.fk(self.q_meas)

        q_goal = self._ik_to(p_start)
        if q_goal is None:
            self._fail(f"IK failed for task start pose {p_start.tolist()}")
            return
        self.approach_start = self.q_meas.copy()
        self.approach_goal = q_goal
        self.get_logger().info(
            f"approach to {task} start p={np.round(p_start, 4).tolist()} "
            f"in {self.args.approach_duration:.1f}s"
        )
        self._start_phase("approach")

    def _begin_task(self, task: str) -> None:
        assert self.q_meas is not None
        self.task_name = task
        duration = float(
            getattr(self.args, "_task_duration", {}).get(task, self.args.duration)
        )
        # Seed IK / FK from measurement
        self.q_cmd = self.q_meas.copy()
        self.q_last_ik = self.q_meas.copy()
        self.dq_cmd = np.zeros(self.nq)
        self.p0, self.R0 = self.kin.fk(self.q_cmd)
        self.traj = build_task_trajectory(
            task,
            self.p0,
            self.R0,
            duration=duration,
            dt=self.dt,
            amplitude_m=self.args.amplitude,
            length_m=self.args.length,
            radius_m=self.args.radius,
            axis=self.args.axis,
            plane=self.args.plane,
            n_rev=self.args.n_rev,
            hold_sec=self.args.hold,
            use_demo_waypoints=self.args.demo_waypoints,
        )
        self.traj_idx = 0
        self.samples = []
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = PROJECT_ROOT / "data" / "mit_traj" / f"{stamp}_{task}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        meta = {
            "task": task,
            "urdf": str(self.kin.urdf_path),
            "ee_frame": self.kin.ee_name,
            "p0": self.p0.tolist(),
            "q0": self.q_cmd.tolist(),
            "demo_waypoints": self.args.demo_waypoints,
            "demo_A": DEMO_POINT_A.tolist(),
            "demo_B": DEMO_POINT_B.tolist(),
            "demo_circle_center": DEMO_CIRCLE_CENTER.tolist(),
            "demo_circle_radius": DEMO_CIRCLE_RADIUS,
            "duration_used": duration,
        }
        (self.run_dir / "meta.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False) + "\n"
        )
        self.get_logger().info(
            f"task={task} n_traj={len(self.traj)} "
            f"p0={[round(x, 4) for x in self.p0]} -> {self.run_dir}"
        )
        self._start_phase("track")

    def _finish_task_fast(self) -> None:
        """Save CSV only; defer plots so we keep streaming commands."""
        assert self.run_dir is not None
        csv_path = self.run_dir / "trajectory.csv"
        write_csv(csv_path, self.samples)
        self.pending_plot_dirs.append(self.run_dir)
        self.run_dirs.append(self.run_dir)

        # Tracking health
        if self.samples:
            q_meas = np.stack([s.q_meas for s in self.samples])
            q_cmd = np.stack([s.q_cmd for s in self.samples])
            q_span = float(np.max(np.ptp(q_meas, axis=0)))
            cmd_span = float(np.max(np.ptp(q_cmd, axis=0)))
            self.get_logger().info(
                f"saved {csv_path} | q_meas_span={q_span:.4f} "
                f"q_cmd_span={cmd_span:.4f} n={len(self.samples)}"
            )
            if cmd_span > 0.05 and q_span < 1e-3:
                self._fail(
                    "Joint feedback frozen (q_meas span≈0 while q_cmd moved). "
                    "Check motors / MIT / serial — aborting remaining tasks."
                )
                return
        self._start_phase("settle_after")

    def _flush_plots(self) -> None:
        for d in self.pending_plot_dirs:
            csv_path = d / "trajectory.csv"
            if not csv_path.is_file():
                continue
            # Keep holding while plotting
            t_end = time.time() + 0.05
            while time.time() < t_end:
                self._hold_current()
                rclpy.spin_once(self, timeout_sec=0.01)
            metrics = write_plots_and_metrics(
                csv_path, d, title_prefix=f"MIT·{d.name}"
            )
            self.get_logger().info(
                f"plots {d.name}: ee_pos_err_mm "
                f"mean={metrics['ee_pos_err_mm_mean']:.2f} "
                f"max={metrics['ee_pos_err_mm_max']:.2f} "
                f"rms={metrics['ee_pos_err_mm_rms']:.2f}"
            )
        self.pending_plot_dirs.clear()

    def _on_timer(self) -> None:
        if self.done or self.q_meas is None:
            return

        if self.phase == "wait_js":
            self._publish(self.q_meas)
            # Need a few JS samples before trusting feedback
            if self._js_count < 5:
                return
            if self.args.skip_home:
                self._begin_approach(self._next_task())
            else:
                self.home_start = self.q_meas.copy()
                self._start_phase("home")
            return

        if self.phase == "home":
            assert self.home_start is not None
            u = min(self._elapsed() / max(self.args.home_duration, 0.1), 1.0)
            s = 0.5 - 0.5 * np.cos(np.pi * u)
            q = self.home_start + s * (self.q_home - self.home_start)
            self._publish(q)
            if u >= 1.0 and self._elapsed() >= self.args.home_duration + self.args.home_settle:
                err = float(np.max(np.abs(self.q_meas - self.q_home)))
                self.get_logger().info(
                    f"home done max_|q-q_home|={err:.4f} moved={self._js_moved}"
                )
                if not self._js_moved and np.linalg.norm(self.home_start - self.q_home) > 0.05:
                    self._fail(
                        "Joint feedback never changed during home — "
                        "motors likely not tracking. Abort."
                    )
                    return
                self.q_last_ik = self.q_meas.copy()
                self._begin_approach(self._next_task())
            return

        if self.phase == "approach":
            assert self.approach_start is not None and self.approach_goal is not None
            u = min(self._elapsed() / max(self.args.approach_duration, 0.1), 1.0)
            s = 0.5 - 0.5 * np.cos(np.pi * u)
            q = self.approach_start + s * (self.approach_goal - self.approach_start)
            self._publish(q)
            if u >= 1.0 and self._elapsed() >= self.args.approach_duration + 0.3:
                self._begin_task(self.task_name)
            return

        if self.phase == "track":
            if self.traj_idx >= len(self.traj):
                self._finish_task_fast()
                return
            pt = self.traj[self.traj_idx]
            self.traj_idx += 1

            q_sol = self._ik_to(pt.p)
            if q_sol is None:
                # keep last command streaming
                self._publish(self.q_cmd)
            else:
                self.dq_cmd = (q_sol - self.q_cmd) / max(self.dt, 1e-6)
                self.q_cmd = q_sol
                self._publish(self.q_cmd)  # velocity=0 like demos

            p_meas, R_meas = self.kin.fk(self.q_meas)
            # Desired orientation not controlled in demo IK — report measured R as cmd R baseline
            self.samples.append(
                PoseSample(
                    t=pt.t,
                    q_cmd=self.q_cmd.copy(),
                    dq_cmd=self.dq_cmd.copy(),
                    q_meas=self.q_meas.copy(),
                    dq_meas=self.dq_meas.copy()
                    if self.dq_meas is not None
                    else np.zeros(self.nq),
                    ee_cmd_xyz=pt.p.copy(),
                    ee_cmd_rpy=_rpy(self.R0),
                    ee_meas_xyz=p_meas.copy(),
                    ee_meas_rpy=_rpy(R_meas),
                )
            )
            return

        if self.phase == "settle_after":
            self._hold_current()
            if self._elapsed() >= self.args.settle:
                nxt = self._next_task()
                if self.args.return_zero:
                    self.zero_start = self.q_meas.copy()
                    self._pending_after_zero = nxt if nxt is not None else ""
                    self._start_phase("return_zero")
                elif nxt is None:
                    self.get_logger().info("all tasks done — writing plots")
                    self._flush_plots()
                    self.done = True
                else:
                    self._begin_approach(nxt)
            return

        if self.phase == "return_zero":
            assert self.zero_start is not None
            u = min(self._elapsed() / max(self.args.zero_duration, 0.1), 1.0)
            s = 0.5 - 0.5 * np.cos(np.pi * u)
            q = self.zero_start + s * (self.q_zero - self.zero_start)
            self._publish(q)
            if u >= 1.0 and self._elapsed() >= self.args.zero_duration + self.args.zero_settle:
                err = float(np.max(np.abs(self.q_meas - self.q_zero)))
                self.get_logger().info(f"return_zero done max_|q|={err:.4f}")
                self.q_last_ik = self.q_meas.copy()
                nxt = self._pending_after_zero
                self._pending_after_zero = None
                if nxt is None or nxt == "":
                    self.get_logger().info("all tasks done — writing plots")
                    self._flush_plots()
                    self.done = True
                else:
                    self._begin_approach(nxt)
            return

    def _next_task(self) -> Optional[str]:
        remaining: list[str] = self.args._remaining_tasks
        if not remaining:
            return None
        return remaining.pop(0)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--task",
        choices=("cosine", "line", "circle", "all"),
        default="cosine",
    )
    p.add_argument("--confirm-hw", action="store_true")
    p.add_argument("--plot-only", action="store_true")
    p.add_argument("--run-dir", type=str, default="")
    p.add_argument("--urdf", type=str, default="")
    p.add_argument("--ee-frame", type=str, default="")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--duration", type=float, default=6.0)
    p.add_argument("--settle", type=float, default=1.0)
    p.add_argument("--amplitude", type=float, default=0.316)
    p.add_argument("--length", type=float, default=0.316)
    p.add_argument("--radius", type=float, default=0.08)
    p.add_argument("--axis", type=str, default="y", choices=("x", "y", "z"))
    p.add_argument("--plane", type=str, default="YZ", choices=("XY", "XZ", "YZ"))
    p.add_argument("--n-rev", type=float, default=1.0)
    p.add_argument("--hold", type=float, default=0.5)
    p.add_argument("--vmax", type=float, default=0.8)
    p.add_argument("--damp", type=float, default=1e-3)
    p.add_argument("--rot-weight", type=float, default=0.25)
    p.add_argument("--skip-home", action="store_true")
    p.add_argument("--home-duration", type=float, default=5.0)
    p.add_argument("--home-settle", type=float, default=0.8)
    p.add_argument("--approach-duration", type=float, default=4.0)
    p.add_argument(
        "--return-zero",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After each task, cosine-ease back to q=0 (default: on)",
    )
    p.add_argument("--zero-duration", type=float, default=5.0)
    p.add_argument("--zero-settle", type=float, default=0.5)
    p.add_argument(
        "--demo-waypoints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use absolute A/B/circle from student demos (default: on)",
    )
    p.add_argument(
        "--zero-velocity",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish velocity=0 like student demos (default: on)",
    )
    p.add_argument(
        "--q-home",
        type=str,
        default=",".join(str(x) for x in DEFAULT_Q_HOME),
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    args.q_home = [float(x) for x in args.q_home.split(",") if x.strip() != ""]
    if len(args.q_home) == 6:
        args.q_home.append(0.0)

    if args.plot_only:
        if not args.run_dir:
            print("--plot-only requires --run-dir", file=sys.stderr)
            return 2
        run_dir = Path(args.run_dir).expanduser().resolve()
        csv_path = run_dir / "trajectory.csv"
        if not csv_path.is_file():
            print(f"missing {csv_path}", file=sys.stderr)
            return 2
        metrics = write_plots_and_metrics(csv_path, run_dir, title_prefix=run_dir.name)
        print(json.dumps(metrics, indent=2, ensure_ascii=False))
        return 0

    if not args.confirm_hw:
        print(
            "Refusing to command hardware without --confirm-hw.\n"
            "Example: python3 scripts/hw_mit_traj_test.py --task all --confirm-hw",
            file=sys.stderr,
        )
        return 2

    if args.task == "all":
        args._remaining_tasks = ["cosine", "line", "circle"]
        args._task_duration = {"cosine": 6.0, "line": 6.0, "circle": 8.0}
    else:
        args._remaining_tasks = [args.task]
        if args.task == "circle" and abs(args.duration - 6.0) < 1e-9:
            args.duration = 8.0
        args._task_duration = {args.task: args.duration}

    rclpy.init()
    node = MitTrajRunner(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted")
    finally:
        if node.pending_plot_dirs:
            try:
                node._flush_plots()
            except Exception as exc:  # noqa: BLE001
                print(f"plot flush failed: {exc}", file=sys.stderr)
        if node.q_meas is not None:
            for _ in range(10):
                node._hold_current()
                time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == "__main__":
    sys.exit(main())
