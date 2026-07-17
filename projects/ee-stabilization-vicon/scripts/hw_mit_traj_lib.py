#!/usr/bin/env python3
"""Shared helpers for hardware MIT trajectory tests (cosine / line / circle).

Provides:
  - Pinocchio FK + damped CLIK (position + light orientation hold)
  - Cartesian trajectory generators
  - CSV I/O and the same-style plots as Mode B failure analysis:
      ee_pose_6panel.png, joints_cmd_fb.png, joints_vel.png
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import pinocchio as pin

_CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if Path(_CJK_FONT).exists():
    fm.fontManager.addfont(_CJK_FONT)
    plt.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
DEFAULT_URDF_CANDIDATES = [
    ARM_ROOT / "windylab_ws" / "src" / "arm-platform" / "config" / "arm.urdf",
    ARM_ROOT / "windylab_ws" / "src" / "arm-platform" / "arm.urdf",
    ARM_ROOT
    / "windylab_ws"
    / "src"
    / "arm_ee_stabilization_description"
    / "urdf"
    / "single_arm.urdf",
]

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
EE_FRAME_CANDIDATES = ("link7", "link6", "ee_link", "tool0")


def resolve_urdf(explicit: Optional[str] = None) -> Path:
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(p)
        return p
    for c in DEFAULT_URDF_CANDIDATES:
        if c.is_file():
            return c
    raise FileNotFoundError("No arm URDF found; pass --urdf")


def resolve_ee_frame(model: pin.Model, preferred: Optional[str] = None) -> int:
    names = [preferred] if preferred else []
    names.extend(EE_FRAME_CANDIDATES)
    for name in names:
        if not name:
            continue
        try:
            fid = model.getFrameId(name)
            if fid < len(model.frames):
                return fid
        except Exception:
            continue
    # last moving frame
    return len(model.frames) - 1


@dataclass
class PoseSample:
    t: float
    q_cmd: np.ndarray
    dq_cmd: np.ndarray
    q_meas: np.ndarray
    dq_meas: np.ndarray
    ee_cmd_xyz: np.ndarray
    ee_cmd_rpy: np.ndarray
    ee_meas_xyz: np.ndarray
    ee_meas_rpy: np.ndarray


@dataclass
class TrajPoint:
    """Desired EE pose / twist in arm base frame."""

    t: float
    p: np.ndarray  # (3,)
    R: np.ndarray  # (3,3)
    v: np.ndarray  # (3,) linear
    w: np.ndarray  # (3,) angular (usually 0)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length axis")
    return v / n


def _rpy(R: np.ndarray) -> np.ndarray:
    return np.array(pin.rpy.matrixToRpy(R), dtype=float)


class ArmKinematics:
    def __init__(self, urdf_path: Path, ee_frame: Optional[str] = None) -> None:
        self.urdf_path = Path(urdf_path)
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()
        self.ee_id = resolve_ee_frame(self.model, ee_frame)
        self.nq = int(self.model.nq)
        self.q_lower = np.array(self.model.lowerPositionLimit, dtype=float)
        self.q_upper = np.array(self.model.upperPositionLimit, dtype=float)
        # replace ±inf with generous bounds
        self.q_lower = np.where(np.isfinite(self.q_lower), self.q_lower, -4.0)
        self.q_upper = np.where(np.isfinite(self.q_upper), self.q_upper, 4.0)

    @property
    def ee_name(self) -> str:
        return self.model.frames[self.ee_id].name

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size < self.nq:
            q = np.pad(q, (0, self.nq - q.size))
        elif q.size > self.nq:
            q = q[: self.nq]
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.ee_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def clik_step(
        self,
        q: np.ndarray,
        p_des: np.ndarray,
        R_des: np.ndarray,
        v_des: np.ndarray,
        w_des: np.ndarray,
        dt: float,
        damp: float = 1e-3,
        rot_weight: float = 0.25,
        vmax: float = 0.8,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """One CLIK step. Returns q_next, dq, p_fk, R_fk."""
        q = np.asarray(q, dtype=float).reshape(-1)[: self.nq].copy()
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.ee_id]
        err_pos = p_des - oMf.translation
        err_rot = pin.log3(R_des @ oMf.rotation.T)
        # task velocity = feedforward + feedback
        kp_pos, kp_rot = 4.0, 2.0
        v_task = np.concatenate(
            [
                v_des + kp_pos * err_pos,
                rot_weight * (w_des + kp_rot * err_rot),
            ]
        )
        J6 = pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_id, pin.LOCAL_WORLD_ALIGNED
        )
        J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + damp * np.eye(JJt.shape[0]), v_task)
        n = float(np.linalg.norm(dq))
        if n > vmax:
            dq *= vmax / n
        q_next = np.clip(q + dq * dt, self.q_lower, self.q_upper)
        p_fk, R_fk = self.fk(q_next)
        return q_next, dq, p_fk, R_fk


def cosine_ease_01(u: float) -> float:
    """u in [0,1] -> smooth 0..1 (half-cosine)."""
    u = float(np.clip(u, 0.0, 1.0))
    return 0.5 - 0.5 * np.cos(np.pi * u)


def cosine_ease_01_deriv(u: float, duration: float) -> float:
    """ds/dt for s=cosine_ease_01(t/T), t=u*T."""
    u = float(np.clip(u, 0.0, 1.0))
    if duration <= 1e-9:
        return 0.0
    return 0.5 * np.pi / duration * np.sin(np.pi * u)


def make_cosine_traj(
    p0: np.ndarray,
    R0: np.ndarray,
    axis: np.ndarray,
    amplitude_m: float,
    duration: float,
    dt: float,
) -> list[TrajPoint]:
    """One-axis go-and-return: s(t)=0.5*(1-cos(2π t/T)), s(0)=s(T)=0, s(T/2)=1."""
    ax = _unit(np.asarray(axis, dtype=float))
    pts: list[TrajPoint] = []
    t = 0.0
    while t <= duration + 1e-9:
        phase = 2.0 * np.pi * (t / max(duration, 1e-9))
        s = 0.5 * (1.0 - np.cos(phase))
        ds_dt = (np.pi / max(duration, 1e-9)) * np.sin(phase)
        p = p0 + amplitude_m * s * ax
        v = amplitude_m * ds_dt * ax
        pts.append(TrajPoint(t=t, p=p, R=R0.copy(), v=v, w=np.zeros(3)))
        t += dt
    return pts


def make_line_traj(
    p0: np.ndarray,
    R0: np.ndarray,
    direction: np.ndarray,
    length_m: float,
    duration: float,
    dt: float,
    hold_sec: float = 0.5,
) -> list[TrajPoint]:
    """Straight line: go with cosine ease, short hold, return with cosine ease."""
    d = _unit(np.asarray(direction, dtype=float))
    half = max(0.5 * (duration - hold_sec), 0.5)
    pts: list[TrajPoint] = []
    t = 0.0
    total = 2.0 * half + hold_sec
    while t <= total + 1e-9:
        if t <= half:
            u = t / half
            s = cosine_ease_01(u)
            ds = cosine_ease_01_deriv(u, half)
            p = p0 + length_m * s * d
            v = length_m * ds * d
        elif t <= half + hold_sec:
            p = p0 + length_m * d
            v = np.zeros(3)
        else:
            u = (t - half - hold_sec) / half
            s = 1.0 - cosine_ease_01(u)
            ds = -cosine_ease_01_deriv(u, half)
            p = p0 + length_m * s * d
            v = length_m * ds * d
        pts.append(TrajPoint(t=t, p=p, R=R0.copy(), v=v, w=np.zeros(3)))
        t += dt
    return pts


def make_circle_traj(
    p0: np.ndarray,
    R0: np.ndarray,
    plane: str,
    radius_m: float,
    duration: float,
    dt: float,
    n_rev: float = 1.0,
) -> list[TrajPoint]:
    """Circle in base frame; continuous at t=0 (passes through p0).

    Center is placed at ``p0 - radius * u`` so the first sample is exactly p0.
    """
    plane = plane.upper()
    if plane == "XY":
        u_ax, v_ax = np.array([1.0, 0, 0]), np.array([0.0, 1, 0])
    elif plane == "XZ":
        u_ax, v_ax = np.array([1.0, 0, 0]), np.array([0.0, 0, 1])
    elif plane == "YZ":
        u_ax, v_ax = np.array([0.0, 1, 0]), np.array([0.0, 0, 1])
    else:
        raise ValueError(f"unknown plane {plane}")
    center = p0 - radius_m * u_ax
    omega = 2.0 * np.pi * n_rev / max(duration, 1e-9)
    pts: list[TrajPoint] = []
    t = 0.0
    while t <= duration + 1e-9:
        th = omega * t
        c, s = np.cos(th), np.sin(th)
        offset = radius_m * (c * u_ax + s * v_ax)
        p = center + offset
        v = radius_m * omega * (-s * u_ax + c * v_ax)
        pts.append(TrajPoint(t=t, p=p, R=R0.copy(), v=v, w=np.zeros(3)))
        t += dt
    return pts


# Absolute waypoints matching arm-platform/demo/move_arm_{line,ik}_demo.py
DEMO_POINT_A = np.array([0.35, -0.15, 0.15])
DEMO_POINT_B = np.array([0.35, 0.15, 0.25])
DEMO_CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
DEMO_CIRCLE_RADIUS = 0.08


def make_demo_cosine_ab(
    duration: float, dt: float, R0: np.ndarray
) -> list[TrajPoint]:
    """Exactly move_arm_line_demo: A↔B with s=0.5*(1-cos(2πt/T))."""
    pts: list[TrajPoint] = []
    t = 0.0
    delta = DEMO_POINT_B - DEMO_POINT_A
    while t <= duration + 1e-9:
        phase = 2.0 * np.pi * (t / max(duration, 1e-9))
        s = 0.5 * (1.0 - np.cos(phase))
        ds_dt = (np.pi / max(duration, 1e-9)) * np.sin(phase)
        p = DEMO_POINT_A + s * delta
        v = ds_dt * delta
        pts.append(TrajPoint(t=t, p=p, R=R0.copy(), v=v, w=np.zeros(3)))
        t += dt
    return pts


def make_demo_line_ab(
    duration: float, dt: float, R0: np.ndarray, hold_sec: float = 0.5
) -> list[TrajPoint]:
    """A→B cosine ease, hold, B→A cosine ease (same endpoints as demo)."""
    delta = DEMO_POINT_B - DEMO_POINT_A
    length = float(np.linalg.norm(delta))
    direction = delta / max(length, 1e-12)
    return make_line_traj(
        DEMO_POINT_A, R0, direction, length, duration, dt, hold_sec
    )


def make_demo_circle(
    duration: float, dt: float, R0: np.ndarray, n_rev: float = 1.0
) -> list[TrajPoint]:
    """Exactly move_arm_ik_demo: YZ circle about DEMO_CIRCLE_CENTER."""
    center = DEMO_CIRCLE_CENTER
    radius = DEMO_CIRCLE_RADIUS
    u_ax, v_ax = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    omega = 2.0 * np.pi * n_rev / max(duration, 1e-9)
    pts: list[TrajPoint] = []
    t = 0.0
    while t <= duration + 1e-9:
        th = omega * t
        c, s = np.cos(th), np.sin(th)
        p = center + radius * (c * u_ax + s * v_ax)
        v = radius * omega * (-s * u_ax + c * v_ax)
        pts.append(TrajPoint(t=t, p=p, R=R0.copy(), v=v, w=np.zeros(3)))
        t += dt
    return pts


def build_task_trajectory(
    task: str,
    p0: np.ndarray,
    R0: np.ndarray,
    *,
    duration: float,
    dt: float,
    amplitude_m: float,
    length_m: float,
    radius_m: float,
    axis: str,
    plane: str,
    n_rev: float,
    hold_sec: float,
    use_demo_waypoints: bool = True,
) -> list[TrajPoint]:
    """Build EE trajectory. Default: absolute student-demo waypoints."""
    task = task.lower()
    if use_demo_waypoints:
        if task == "cosine":
            return make_demo_cosine_ab(duration, dt, R0)
        if task == "line":
            return make_demo_line_ab(duration, dt, R0, hold_sec)
        if task == "circle":
            return make_demo_circle(duration, dt, R0, n_rev)
        raise ValueError(f"unknown task {task}")

    ax_map = {
        "x": np.array([1.0, 0, 0]),
        "y": np.array([0.0, 1, 0]),
        "z": np.array([0.0, 0, 1]),
    }
    if task == "cosine":
        return make_cosine_traj(
            p0, R0, ax_map[axis.lower()], amplitude_m, duration, dt
        )
    if task == "line":
        return make_line_traj(
            p0, R0, ax_map[axis.lower()], length_m, duration, dt, hold_sec
        )
    if task == "circle":
        # relative circle still honors radius_m / plane
        return make_circle_traj(
            p0, R0, plane, radius_m, duration, dt, n_rev
        )
    raise ValueError(f"unknown task {task}")


CSV_FIELDS = [
    "t",
    *[f"q_cmd_{i}" for i in range(1, 8)],
    *[f"dq_cmd_{i}" for i in range(1, 8)],
    *[f"q_meas_{i}" for i in range(1, 8)],
    *[f"dq_meas_{i}" for i in range(1, 8)],
    "ee_cmd_x",
    "ee_cmd_y",
    "ee_cmd_z",
    "ee_cmd_roll",
    "ee_cmd_pitch",
    "ee_cmd_yaw",
    "ee_meas_x",
    "ee_meas_y",
    "ee_meas_z",
    "ee_meas_roll",
    "ee_meas_pitch",
    "ee_meas_yaw",
]


def pad7(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float).reshape(-1)
    out = np.zeros(7)
    out[: min(7, a.size)] = a[: min(7, a.size)]
    return out


def samples_to_rows(samples: Iterable[PoseSample]) -> list[dict]:
    rows = []
    for s in samples:
        qc, dqc = pad7(s.q_cmd), pad7(s.dq_cmd)
        qm, dqm = pad7(s.q_meas), pad7(s.dq_meas)
        row = {"t": f"{s.t:.6f}"}
        for i in range(7):
            row[f"q_cmd_{i+1}"] = f"{qc[i]:.8f}"
            row[f"dq_cmd_{i+1}"] = f"{dqc[i]:.8f}"
            row[f"q_meas_{i+1}"] = f"{qm[i]:.8f}"
            row[f"dq_meas_{i+1}"] = f"{dqm[i]:.8f}"
        for name, arr in (
            ("ee_cmd", np.concatenate([s.ee_cmd_xyz, s.ee_cmd_rpy])),
            ("ee_meas", np.concatenate([s.ee_meas_xyz, s.ee_meas_rpy])),
        ):
            for k, key in enumerate(
                ("x", "y", "z", "roll", "pitch", "yaw")
            ):
                row[f"{name}_{key}"] = f"{arr[k]:.8f}"
        rows.append(row)
    return rows


def write_csv(path: Path, samples: list[PoseSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for row in samples_to_rows(samples):
            w.writerow(row)


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty csv: {path}")

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=float)

    data = {"t": col("t")}
    for i in range(1, 8):
        data[f"q_cmd_{i}"] = col(f"q_cmd_{i}")
        data[f"dq_cmd_{i}"] = col(f"dq_cmd_{i}")
        data[f"q_meas_{i}"] = col(f"q_meas_{i}")
        data[f"dq_meas_{i}"] = col(f"dq_meas_{i}")
    for prefix in ("ee_cmd", "ee_meas"):
        for k in ("x", "y", "z", "roll", "pitch", "yaw"):
            data[f"{prefix}_{k}"] = col(f"{prefix}_{k}")
    return data


def plot_ee_pose_6panel(data: dict[str, np.ndarray], out_path: Path, title: str) -> None:
    t = data["t"]
    labels = [
        ("x", "X (m)", False),
        ("y", "Y (m)", False),
        ("z", "Z (m)", False),
        ("roll", "Roll (rad)", True),
        ("pitch", "Pitch (rad)", True),
        ("yaw", "Yaw (rad)", True),
    ]
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    axes = axes.ravel()
    for ax, (key, ylab, _) in zip(axes, labels):
        ax.plot(t, data[f"ee_cmd_{key}"], "k--", lw=1.4, label="目标")
        ax.plot(t, data[f"ee_meas_{key}"], "C0", lw=1.2, label="实际")
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axes[-2].set_xlabel("t (s)")
    axes[-1].set_xlabel("t (s)")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_joints_cmd_fb(data: dict[str, np.ndarray], out_path: Path, title: str) -> None:
    t = data["t"]
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    axes = axes.ravel()
    for i in range(7):
        ax = axes[i]
        ax.plot(t, data[f"q_cmd_{i+1}"], "k--", lw=1.3, label="q_cmd")
        ax.plot(t, data[f"q_meas_{i+1}"], "C0", lw=1.1, label="q_fb")
        ax.set_ylabel(f"j{i+1} (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axes[7].axis("off")
    axes[5].set_xlabel("t (s)")
    axes[6].set_xlabel("t (s)")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_joints_vel(data: dict[str, np.ndarray], out_path: Path, title: str) -> None:
    t = data["t"]
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    axes = axes.ravel()
    for i in range(7):
        ax = axes[i]
        ax.plot(t, data[f"dq_cmd_{i+1}"], "k--", lw=1.2, label="dq_cmd")
        ax.plot(t, data[f"dq_meas_{i+1}"], "C1", lw=1.0, label="dq_fb")
        ax.set_ylabel(f"j{i+1} (rad/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axes[7].axis("off")
    axes[5].set_xlabel("t (s)")
    axes[6].set_xlabel("t (s)")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def compute_metrics(data: dict[str, np.ndarray]) -> dict:
    pos_err = np.sqrt(
        (data["ee_cmd_x"] - data["ee_meas_x"]) ** 2
        + (data["ee_cmd_y"] - data["ee_meas_y"]) ** 2
        + (data["ee_cmd_z"] - data["ee_meas_z"]) ** 2
    )
    q_err = []
    for i in range(1, 8):
        q_err.append(np.abs(data[f"q_cmd_{i}"] - data[f"q_meas_{i}"]))
    q_err = np.stack(q_err, axis=0)
    return {
        "n_samples": int(data["t"].size),
        "duration_s": float(data["t"][-1] - data["t"][0]) if data["t"].size else 0.0,
        "ee_pos_err_mm_mean": float(np.mean(pos_err) * 1000.0),
        "ee_pos_err_mm_max": float(np.max(pos_err) * 1000.0),
        "ee_pos_err_mm_rms": float(np.sqrt(np.mean(pos_err**2)) * 1000.0),
        "joint_abs_err_rad_mean": [float(np.mean(q_err[i])) for i in range(7)],
        "joint_abs_err_rad_max": [float(np.max(q_err[i])) for i in range(7)],
    }


def write_plots_and_metrics(
    csv_path: Path, out_dir: Path, title_prefix: str
) -> dict:
    data = load_csv(csv_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_ee_pose_6panel(
        data, out_dir / "ee_pose_6panel.png", f"{title_prefix} · 末端位姿 目标 vs 实际"
    )
    plot_joints_cmd_fb(
        data, out_dir / "joints_cmd_fb.png", f"{title_prefix} · 关节 q_cmd vs q_fb"
    )
    plot_joints_vel(
        data, out_dir / "joints_vel.png", f"{title_prefix} · 关节速度"
    )
    metrics = compute_metrics(data)
    (out_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n"
    )
    return metrics
