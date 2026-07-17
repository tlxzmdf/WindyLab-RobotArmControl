#!/usr/bin/env python3
"""MIT j1–j3 kp/kd 调参：参数、overlay YAML、轨迹打分、搜索建议。

增益只在 student_arm_node 启动时加载，每轮需重启节点。
ROS2 double_array 必须写成浮点字面量（30.0 而非 30）。
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from itertools import product
from pathlib import Path
from typing import Any, Optional

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TUNE_ROOT = PROJECT_ROOT / "data" / "tune_mit_gains"

# Distal joints kept at stock MIT defaults
DISTAL_KP = (5.0, 5.0, 5.0, 1.0)  # j4..j7
DISTAL_KD = (0.1, 0.1, 0.1, 0.1)

DEFAULT_GRID: dict[str, list[float]] = {
    "kp123": [40.0, 50.0, 60.0, 70.0],
    "kd123": [1.0, 1.5, 2.0, 2.5],
}

DEFAULT_WEIGHTS = {
    "w_ee_rms": 1.0,
    "w_ee_max": 0.6,
    "w_j123_mean": 80.0,  # rad → roughly mm-scale contribution
    "w_chatter": 25.0,  # high-freq joint activity
    "w_invalid": 1e6,
}


def _clip(x: float, lo: float, hi: float) -> float:
    return float(min(hi, max(lo, x)))


def _yaml_float(x: float) -> str:
    s = f"{float(x):.6f}".rstrip("0").rstrip(".")
    if "." not in s and "e" not in s.lower():
        s += ".0"
    return s


@dataclass
class MitProxGains:
    """Proximal (j1–j3) MIT gains; j4–j7 fixed to stock."""

    kp123: float = 60.0
    kd123: float = 1.5
    # optional per-joint overrides (None → use kp123/kd123)
    kp1: Optional[float] = None
    kp2: Optional[float] = None
    kp3: Optional[float] = None
    kd1: Optional[float] = None
    kd2: Optional[float] = None
    kd3: Optional[float] = None
    max_velocity: float = 0.5
    command_timeout_sec: float = 2.0
    torque_limit: float = 9.0

    def clamp(self) -> "MitProxGains":
        return MitProxGains(
            kp123=_clip(self.kp123, 20.0, 80.0),
            kd123=_clip(self.kd123, 0.3, 5.0),
            kp1=None if self.kp1 is None else _clip(self.kp1, 20.0, 80.0),
            kp2=None if self.kp2 is None else _clip(self.kp2, 20.0, 80.0),
            kp3=None if self.kp3 is None else _clip(self.kp3, 20.0, 80.0),
            kd1=None if self.kd1 is None else _clip(self.kd1, 0.3, 5.0),
            kd2=None if self.kd2 is None else _clip(self.kd2, 0.3, 5.0),
            kd3=None if self.kd3 is None else _clip(self.kd3, 0.3, 5.0),
            max_velocity=_clip(self.max_velocity, 0.2, 1.0),
            command_timeout_sec=_clip(self.command_timeout_sec, 0.5, 5.0),
            torque_limit=_clip(self.torque_limit, 3.0, 9.0),
        )

    def p_gain(self) -> list[float]:
        p = self.clamp()
        k1 = p.kp1 if p.kp1 is not None else p.kp123
        k2 = p.kp2 if p.kp2 is not None else p.kp123
        k3 = p.kp3 if p.kp3 is not None else p.kp123
        return [k1, k2, k3, *DISTAL_KP]

    def d_gain(self) -> list[float]:
        p = self.clamp()
        d1 = p.kd1 if p.kd1 is not None else p.kd123
        d2 = p.kd2 if p.kd2 is not None else p.kd123
        d3 = p.kd3 if p.kd3 is not None else p.kd123
        return [d1, d2, d3, *DISTAL_KD]

    def key(self) -> str:
        p = self.clamp()
        pg = self.p_gain()
        dg = self.d_gain()
        return (
            f"kp{pg[0]:.0f}_{pg[1]:.0f}_{pg[2]:.0f}"
            f"_kd{dg[0]:.1f}_{dg[1]:.1f}_{dg[2]:.1f}"
        ).replace(".", "p")

    def to_dict(self) -> dict[str, Any]:
        p = self.clamp()
        return {
            "kp123": p.kp123,
            "kd123": p.kd123,
            "kp1": p.kp1,
            "kp2": p.kp2,
            "kp3": p.kp3,
            "kd1": p.kd1,
            "kd2": p.kd2,
            "kd3": p.kd3,
            "p_gain": self.p_gain(),
            "d_gain": self.d_gain(),
            "max_velocity": p.max_velocity,
            "command_timeout_sec": p.command_timeout_sec,
            "torque_limit": p.torque_limit,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "MitProxGains":
        return cls(
            kp123=float(d.get("kp123", 30.0)),
            kd123=float(d.get("kd123", 1.0)),
            kp1=_optf(d.get("kp1")),
            kp2=_optf(d.get("kp2")),
            kp3=_optf(d.get("kp3")),
            kd1=_optf(d.get("kd1")),
            kd2=_optf(d.get("kd2")),
            kd3=_optf(d.get("kd3")),
            max_velocity=float(d.get("max_velocity", 0.5)),
            command_timeout_sec=float(d.get("command_timeout_sec", 2.0)),
            torque_limit=float(d.get("torque_limit", 9.0)),
        ).clamp()


def _optf(v: Any) -> Optional[float]:
    if v is None or v == "":
        return None
    return float(v)


def write_student_overlay_yaml(path: Path, gains: MitProxGains) -> Path:
    """Write student_arm_node params overlay (floats only)."""
    g = gains.clamp()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pg = ", ".join(_yaml_float(x) for x in g.p_gain())
    dg = ", ".join(_yaml_float(x) for x in g.d_gain())
    lines = [
        "# Auto-generated MIT proximal-gain overlay — floats required for ROS.",
        "student_arm_node:",
        "  ros__parameters:",
        "    controller_type: mit_stabilization",
        f"    p_gain: [{pg}]",
        f"    d_gain: [{dg}]",
        f"    max_velocity: {_yaml_float(g.max_velocity)}",
        f"    command_timeout_sec: {_yaml_float(g.command_timeout_sec)}",
        f"    torque_limit: {_yaml_float(g.torque_limit)}",
        "    publish_joint_feedback: true",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


@dataclass
class MitTuneScore:
    run_dir: str
    valid: bool
    score: float  # lower better
    reason: str = ""
    ee_rms_mm: float = float("nan")
    ee_mean_mm: float = float("nan")
    ee_max_mm: float = float("nan")
    j123_mean_rad: float = float("nan")
    j1_mean_rad: float = float("nan")
    chatter: float = float("nan")
    q_meas_span: float = float("nan")
    q_cmd_span: float = float("nan")
    extras: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def score_trajectory_csv(
    csv_path: Path,
    weights: Optional[dict[str, float]] = None,
) -> MitTuneScore:
    w = {**DEFAULT_WEIGHTS, **(weights or {})}
    csv_path = Path(csv_path)
    if not csv_path.is_file():
        return MitTuneScore(
            run_dir=str(csv_path.parent),
            valid=False,
            score=w["w_invalid"],
            reason="missing trajectory.csv",
        )

    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 20:
        return MitTuneScore(
            run_dir=str(csv_path.parent),
            valid=False,
            score=w["w_invalid"],
            reason=f"too few samples ({len(rows)})",
        )

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=float)

    t = col("t")
    err = np.sqrt(
        (col("ee_cmd_x") - col("ee_meas_x")) ** 2
        + (col("ee_cmd_y") - col("ee_meas_y")) ** 2
        + (col("ee_cmd_z") - col("ee_meas_z")) ** 2
    )
    ee_rms = float(np.sqrt(np.mean(err**2)) * 1e3)
    ee_mean = float(np.mean(err) * 1e3)
    ee_max = float(np.max(err) * 1e3)

    j_err = []
    q_cmd_span = []
    q_meas_span = []
    for i in range(1, 8):
        qc, qm = col(f"q_cmd_{i}"), col(f"q_meas_{i}")
        j_err.append(np.abs(qc - qm))
        q_cmd_span.append(float(np.ptp(qc)))
        q_meas_span.append(float(np.ptp(qm)))
    j123_mean = float(np.mean([np.mean(j_err[i]) for i in range(3)]))
    j1_mean = float(np.mean(j_err[0]))
    cmd_span = float(max(q_cmd_span))
    meas_span = float(max(q_meas_span))

    # Chatter: RMS of discrete accel on j1–j3 measured positions
    dt = float(np.median(np.diff(t))) if t.size > 1 else 0.02
    chatter_terms = []
    for i in range(3):
        qm = col(f"q_meas_{i+1}")
        if qm.size < 5:
            continue
        ddq = np.diff(qm, n=2) / max(dt**2, 1e-6)
        chatter_terms.append(float(np.sqrt(np.mean(ddq**2))))
    chatter = float(np.mean(chatter_terms)) if chatter_terms else 0.0

    if cmd_span > 0.05 and meas_span < 1e-3:
        return MitTuneScore(
            run_dir=str(csv_path.parent),
            valid=False,
            score=w["w_invalid"],
            reason="q_meas frozen while q_cmd moved",
            ee_rms_mm=ee_rms,
            ee_mean_mm=ee_mean,
            ee_max_mm=ee_max,
            j123_mean_rad=j123_mean,
            j1_mean_rad=j1_mean,
            chatter=chatter,
            q_meas_span=meas_span,
            q_cmd_span=cmd_span,
        )

    score = (
        w["w_ee_rms"] * ee_rms
        + w["w_ee_max"] * ee_max
        + w["w_j123_mean"] * j123_mean
        + w["w_chatter"] * chatter
    )
    return MitTuneScore(
        run_dir=str(csv_path.parent),
        valid=True,
        score=float(score),
        reason="ok",
        ee_rms_mm=ee_rms,
        ee_mean_mm=ee_mean,
        ee_max_mm=ee_max,
        j123_mean_rad=j123_mean,
        j1_mean_rad=j1_mean,
        chatter=chatter,
        q_meas_span=meas_span,
        q_cmd_span=cmd_span,
        extras={"n": len(rows), "dt": dt},
    )


def score_multiple_runs(
    run_dirs: list[Path],
    weights: Optional[dict[str, float]] = None,
) -> MitTuneScore:
    """Aggregate scores across cosine/line/circle (mean of per-task scores)."""
    w = {**DEFAULT_WEIGHTS, **(weights or {})}
    if not run_dirs:
        return MitTuneScore(
            run_dir="",
            valid=False,
            score=w["w_invalid"],
            reason="no_run_dirs",
        )
    per: list[MitTuneScore] = []
    for d in run_dirs:
        per.append(score_trajectory_csv(Path(d) / "trajectory.csv", weights))
    invalid = [s for s in per if not s.valid]
    if invalid:
        return MitTuneScore(
            run_dir=",".join(str(Path(d)) for d in run_dirs),
            valid=False,
            score=w["w_invalid"],
            reason="invalid:" + ",".join(s.reason for s in invalid),
            extras={"per_task": [s.to_dict() for s in per]},
        )
    return MitTuneScore(
        run_dir=",".join(str(Path(d)) for d in run_dirs),
        valid=True,
        score=float(np.mean([s.score for s in per])),
        reason="ok_all_tasks",
        ee_rms_mm=float(np.mean([s.ee_rms_mm for s in per])),
        ee_mean_mm=float(np.mean([s.ee_mean_mm for s in per])),
        ee_max_mm=float(np.max([s.ee_max_mm for s in per])),
        j123_mean_rad=float(np.mean([s.j123_mean_rad for s in per])),
        j1_mean_rad=float(np.mean([s.j1_mean_rad for s in per])),
        chatter=float(np.mean([s.chatter for s in per])),
        q_meas_span=float(np.mean([s.q_meas_span for s in per])),
        q_cmd_span=float(np.mean([s.q_cmd_span for s in per])),
        extras={
            "per_task": [s.to_dict() for s in per],
            "task_scores": [s.score for s in per],
        },
    )


def expand_grid(grid: Optional[dict[str, list[float]]] = None) -> list[MitProxGains]:
    g = grid or DEFAULT_GRID
    keys = sorted(g.keys())
    out: list[MitProxGains] = []
    for vals in product(*(g[k] for k in keys)):
        kwargs = {k: float(v) for k, v in zip(keys, vals)}
        out.append(MitProxGains(**kwargs).clamp())
    # dedupe by key
    seen = set()
    uniq = []
    for p in out:
        k = p.key()
        if k in seen:
            continue
        seen.add(k)
        uniq.append(p)
    return uniq


def append_trial_log(log_path: Path, trial: dict[str, Any]) -> None:
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(trial, ensure_ascii=False) + "\n")


def load_trial_log(log_path: Path) -> list[dict[str, Any]]:
    log_path = Path(log_path)
    if not log_path.is_file():
        return []
    rows = []
    for line in log_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


def best_from_log(log_path: Path) -> Optional[dict[str, Any]]:
    rows = [r for r in load_trial_log(log_path) if r.get("valid") and math.isfinite(r.get("score", float("nan")))]
    if not rows:
        return None
    return min(rows, key=lambda r: float(r["score"]))


def suggest_next(
    log_path: Path,
    strategy: str = "coord",
    grid: Optional[dict[str, list[float]]] = None,
) -> MitProxGains:
    """Pick next untested gains. coord: alternate kp/kd 1D search around best."""
    grid = grid or DEFAULT_GRID
    tried = {r.get("key") for r in load_trial_log(log_path)}
    best = best_from_log(log_path)

    if strategy == "grid":
        for g in expand_grid(grid):
            if g.key() not in tried:
                return g
        # all done → return best or default
        if best:
            return MitProxGains.from_dict(best.get("gains", {}))
        return MitProxGains()

    # coordinate descent around best (or baseline)
    base = MitProxGains.from_dict(best["gains"]) if best else MitProxGains()
    # phase: prefer exploring the dimension with fewer trials
    kp_opts = list(grid.get("kp123", DEFAULT_GRID["kp123"]))
    kd_opts = list(grid.get("kd123", DEFAULT_GRID["kd123"]))

    # First fill neighbors of best kp at fixed kd, then kd at fixed kp
    candidates: list[MitProxGains] = []
    for kp in kp_opts:
        candidates.append(MitProxGains(kp123=kp, kd123=base.kd123).clamp())
    for kd in kd_opts:
        candidates.append(MitProxGains(kp123=base.kp123, kd123=kd).clamp())
    # then full remaining grid
    candidates.extend(expand_grid(grid))

    for c in candidates:
        if c.key() not in tried:
            return c
    return base


def format_score_table(rows: list[dict[str, Any]], top: int = 12) -> str:
    valid = [r for r in rows if r.get("valid")]
    valid.sort(key=lambda r: float(r.get("score", 1e9)))
    lines = [
        f"{'rank':>4} {'key':<28} {'score':>8} {'ee_rms':>7} {'ee_max':>7} {'j123':>7} {'chatter':>8}",
        "-" * 78,
    ]
    for i, r in enumerate(valid[:top], 1):
        lines.append(
            f"{i:4d} {r.get('key', ''):<28} {float(r['score']):8.2f} "
            f"{float(r.get('ee_rms_mm', float('nan'))):7.2f} "
            f"{float(r.get('ee_max_mm', float('nan'))):7.2f} "
            f"{float(r.get('j123_mean_rad', float('nan'))):7.4f} "
            f"{float(r.get('chatter', float('nan'))):8.2f}"
        )
    if not valid:
        lines.append("(no valid trials yet)")
    return "\n".join(lines)
