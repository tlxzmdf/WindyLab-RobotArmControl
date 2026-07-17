#!/usr/bin/env python3
"""Mode C 调参：参数空间、YAML overlay、run 打分。"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TUNE_ROOT = PROJECT_ROOT / "data" / "tune"


@dataclass
class ModeCParams:
    """可调 Mode C 增益（写入 ee_stabilization overlay）。"""

    kp_pos: float = 600.0
    kp_ori: float = 450.0
    kd_pos: float = 100.0
    kd_ori: float = 130.0
    osc_lambda: float = 0.05
    hw_torque_lpf_alpha: float = 0.55
    hw_torque_limit: float = 6.0
    hw_zero_dq: bool = True

    def clamp(self) -> "ModeCParams":
        return ModeCParams(
            kp_pos=_clip(self.kp_pos, 300.0, 900.0),
            kp_ori=_clip(self.kp_ori, 200.0, 700.0),
            kd_pos=_clip(self.kd_pos, 60.0, 160.0),
            kd_ori=_clip(self.kd_ori, 80.0, 200.0),
            osc_lambda=_clip(self.osc_lambda, 0.02, 0.15),
            hw_torque_lpf_alpha=_clip(self.hw_torque_lpf_alpha, 0.35, 0.90),
            hw_torque_limit=_clip(self.hw_torque_limit, 4.0, 9.0),
            hw_zero_dq=bool(self.hw_zero_dq),
        )

    def key(self) -> str:
        p = self.clamp()
        return (
            f"kp{p.kp_pos:.0f}_{p.kp_ori:.0f}"
            f"_kd{p.kd_pos:.0f}_{p.kd_ori:.0f}"
            f"_lam{p.osc_lambda:.3f}"
            f"_a{p.hw_torque_lpf_alpha:.2f}"
        )

    def to_ros_dict(self) -> dict[str, Any]:
        p = self.clamp()
        return {
            "kp_task": [p.kp_pos, p.kp_pos, p.kp_pos, p.kp_ori, p.kp_ori, p.kp_ori],
            "kd_task": [p.kd_pos, p.kd_pos, p.kd_pos, p.kd_ori, p.kd_ori, p.kd_ori],
            "osc_lambda": p.osc_lambda,
            "hw_torque_lpf_alpha": p.hw_torque_lpf_alpha,
            "hw_torque_limit": p.hw_torque_limit,
            "hw_zero_dq": p.hw_zero_dq,
            "use_ik_joint_control": False,
            "kinematic_stabilization": False,
            "hardware_mode": True,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "ModeCParams":
        return cls(
            kp_pos=float(d.get("kp_pos", 600.0)),
            kp_ori=float(d.get("kp_ori", 450.0)),
            kd_pos=float(d.get("kd_pos", 100.0)),
            kd_ori=float(d.get("kd_ori", 130.0)),
            osc_lambda=float(d.get("osc_lambda", 0.05)),
            hw_torque_lpf_alpha=float(d.get("hw_torque_lpf_alpha", 0.55)),
            hw_torque_limit=float(d.get("hw_torque_limit", 6.0)),
            hw_zero_dq=bool(d.get("hw_zero_dq", True)),
        ).clamp()


@dataclass
class TuneScore:
    run_dir: str
    valid: bool
    score: float  # lower is better
    reason: str = ""
    duration_s: float = 0.0
    e_quiet_rms_mm: float = float("nan")
    e_strong_med_mm: float = float("nan")  # Δ 80–140 mm
    ratio_strong_med: float = float("nan")
    e_static_med_mm: float = float("nan")  # post-disturb, Δ<10
    delta_max_mm: float = float("nan")
    e_max_mm: float = float("nan")
    v12_rms: float = float("nan")
    edt_rms_mm_s: float = float("nan")
    tau_sat_frac: float = float("nan")
    tf_ok_frac: float = float("nan")
    n_strong: int = 0
    n_static: int = 0
    extras: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        return d


# 默认搜索网格（可被 CLI / JSON 覆盖）
DEFAULT_GRID: dict[str, list[float]] = {
    "kp_pos": [420.0, 520.0, 600.0, 700.0, 800.0],
    "kp_ori": [320.0, 400.0, 450.0, 520.0],
    "kd_pos": [90.0, 100.0, 110.0],
    "kd_ori": [120.0, 130.0, 140.0],
    "osc_lambda": [0.03, 0.05, 0.08, 0.10],
    "hw_torque_lpf_alpha": [0.45, 0.55, 0.65, 0.75],
}

# 打分权重（可按场景改）
DEFAULT_WEIGHTS = {
    "w_strong_e": 1.0,  # 大扰动位置误差
    "w_static_e": 1.5,  # 回正静差（主目标）
    "w_ratio": 8.0,  # e/Δ
    "w_chatter": 12.0,  # |v| j1-2
    "w_edt": 0.02,  # |ė| mm/s
    "w_sat": 40.0,  # τ 顶满占比
    "w_invalid": 1e6,
}


def _clip(x: float, lo: float, hi: float) -> float:
    return float(min(hi, max(lo, x)))


def _getf(row: dict[str, str], *keys: str) -> float:
    for k in keys:
        v = row.get(k, "")
        if v in ("", None):
            continue
        try:
            x = float(v)
            if x == x:
                return x
        except (TypeError, ValueError):
            pass
    return float("nan")


def _yaml_float(x: float) -> str:
    """Always emit a YAML float token (ROS needs double_array, not integer_array)."""
    s = f"{float(x):.6f}".rstrip("0").rstrip(".")
    if "." not in s and "e" not in s.lower():
        s += ".0"
    return s


def write_overlay_yaml(path: Path, params: ModeCParams) -> Path:
    """写 ROS2 参数 overlay（最后合并进 ee_stabilization）。"""
    p = params.clamp()
    d = p.to_ros_dict()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Auto-generated Mode C overlay — do not edit by hand during a tune session.",
        "ee_stabilization:",
        "  ros__parameters:",
    ]
    for k, v in d.items():
        if isinstance(v, bool):
            lines.append(f"    {k}: {'true' if v else 'false'}")
        elif isinstance(v, list):
            inner = ", ".join(_yaml_float(float(x)) for x in v)
            lines.append(f"    {k}: [{inner}]")
        elif isinstance(v, float):
            lines.append(f"    {k}: {_yaml_float(v)}")
        else:
            lines.append(f"    {k}: {v}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def expand_grid(
    grid: Optional[dict[str, list[float]]] = None,
    base: Optional[ModeCParams] = None,
    max_points: int = 64,
) -> list[ModeCParams]:
    """笛卡尔积网格；过大时对次要轴做抽样。"""
    g = {k: list(v) for k, v in (grid or DEFAULT_GRID).items()}
    base = (base or ModeCParams()).clamp()

    # 完整积可能很大：优先扫 kp_pos × lambda × alpha，其余用 base
    keys_full = ["kp_pos", "osc_lambda", "hw_torque_lpf_alpha"]
    keys_opt = ["kp_ori", "kd_pos", "kd_ori"]

    from itertools import product

    full_axes = [g.get(k, [getattr(base, k)]) for k in keys_full]
    candidates: list[ModeCParams] = []
    for vals in product(*full_axes):
        kw = {k: float(v) for k, v in zip(keys_full, vals)}
        for k in keys_opt:
            kw[k] = float(getattr(base, k))
        candidates.append(ModeCParams(**kw).clamp())

    # 若仍有空间，追加 kp_ori / kd 一维扰动
    extras: list[ModeCParams] = []
    for k in keys_opt:
        for v in g.get(k, []):
            kw = asdict(base)
            kw[k] = float(v)
            extras.append(ModeCParams(**kw).clamp())

    seen: set[str] = set()
    out: list[ModeCParams] = []
    for p in candidates + extras:
        key = p.key()
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
        if len(out) >= max_points:
            break
    return out


def random_candidates(
    n: int,
    base: Optional[ModeCParams] = None,
    seed: int = 0,
) -> list[ModeCParams]:
    rng = np.random.default_rng(seed)
    base = (base or ModeCParams()).clamp()
    out: list[ModeCParams] = []
    for _ in range(n):
        out.append(
            ModeCParams(
                kp_pos=float(rng.uniform(400, 850)),
                kp_ori=float(rng.uniform(300, 600)),
                kd_pos=float(rng.uniform(80, 120)),
                kd_ori=float(rng.uniform(110, 150)),
                osc_lambda=float(rng.uniform(0.03, 0.12)),
                hw_torque_lpf_alpha=float(rng.uniform(0.45, 0.75)),
                hw_torque_limit=base.hw_torque_limit,
                hw_zero_dq=True,
            ).clamp()
        )
    return out


def coordinate_next(
    best: ModeCParams,
    axis: str,
    grid: Optional[dict[str, list[float]]] = None,
    tried: Optional[set[str]] = None,
) -> Optional[ModeCParams]:
    """沿单一轴取下一个未试过的网格点（优先靠近当前 best）。"""
    g = grid or DEFAULT_GRID
    tried = tried or set()
    values = g.get(axis)
    if not values:
        return None
    cur = float(getattr(best, axis))
    ordered = sorted(values, key=lambda v: (abs(float(v) - cur), float(v)))
    for v in ordered:
        if abs(float(v) - cur) < 1e-9:
            continue  # skip current best on this axis
        kw = asdict(best)
        kw[axis] = float(v)
        p = ModeCParams(**kw).clamp()
        if p.key() not in tried:
            return p
    return None


def score_aligned_csv(
    run_dir: Path,
    weights: Optional[dict[str, float]] = None,
) -> TuneScore:
    """从 data/runs/<id>/aligned.csv 打分（越低越好）。"""
    w = {**DEFAULT_WEIGHTS, **(weights or {})}
    run_dir = Path(run_dir)
    csv_path = run_dir / "aligned.csv"
    if not csv_path.is_file():
        return TuneScore(
            run_dir=str(run_dir),
            valid=False,
            score=w["w_invalid"],
            reason="missing aligned.csv",
        )

    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    if len(rows) < 50:
        return TuneScore(
            run_dir=str(run_dir),
            valid=False,
            score=w["w_invalid"],
            reason=f"too few samples ({len(rows)})",
        )

    t = np.array([_getf(r, "t_sec") for r in rows], float)
    e = np.array([_getf(r, "world_pos_err_m") for r in rows], float)
    d = np.array([_getf(r, "delta_pos_norm") for r in rows], float)
    tf = np.array(
        [
            _getf(r, "tf_base_ok", "tf_ok")
            if any(k in r for k in ("tf_base_ok", "tf_ok"))
            else 1.0
            for r in rows
        ],
        float,
    )
    # fallback: presence of delta
    if not np.any(np.isfinite(tf)):
        tf = np.isfinite(d).astype(float)

    v1 = np.array([_getf(r, "joint1_fb_vel", "joint1_vel") for r in rows], float)
    v2 = np.array([_getf(r, "joint2_fb_vel", "joint2_vel") for r in rows], float)
    v12 = np.sqrt(np.nan_to_num(v1) ** 2 + np.nan_to_num(v2) ** 2)

    taus = []
    for j in range(1, 8):
        taus.append(
            np.array(
                [_getf(r, f"joint{j}_tau_ff", f"joint{j}_eff") for r in rows], float
            )
        )
    tau_stack = np.stack([np.nan_to_num(ta) for ta in taus], axis=0)
    sat = np.any(np.abs(tau_stack) >= 5.9, axis=0).astype(float)

    finite = np.isfinite(t) & np.isfinite(e) & np.isfinite(d)
    if finite.sum() < 50:
        return TuneScore(
            run_dir=str(run_dir),
            valid=False,
            score=w["w_invalid"],
            reason="non-finite metrics",
        )

    t, e, d, v12, sat, tf = t[finite], e[finite], d[finite], v12[finite], sat[finite], tf[finite]
    duration = float(t[-1] - t[0])
    edt = np.gradient(e, t)

    # early quiet (~ first 2 s or until Δ grows)
    early = t <= (t[0] + 2.0)
    strong = (d >= 0.08) & (d <= 0.14)
    if strong.sum() < 30:
        strong = d >= 0.05  # fallback wider band

    # first strong disturb time
    strong_any = np.where(d >= 0.05)[0]
    if len(strong_any) == 0:
        return TuneScore(
            run_dir=str(run_dir),
            valid=False,
            score=w["w_invalid"] * 0.5,
            reason="no disturbance (Δ never ≥50mm) — shake the aircraft",
            duration_s=duration,
            delta_max_mm=float(np.nanmax(d) * 1000),
            e_max_mm=float(np.nanmax(e) * 1000),
            tf_ok_frac=float(np.nanmean(tf)),
        )

    t_dist0 = t[strong_any[0]]
    static = (t > t_dist0) & (d < 0.010)
    # if almost no return-to-origin samples, use late window when Δ small-ish
    if static.sum() < 40:
        late = t >= (t[-1] - 5.0)
        static_alt = late & (d < 0.03)
        if static_alt.sum() >= 40:
            static = static_alt

    def _med(x: np.ndarray) -> float:
        x = x[np.isfinite(x)]
        return float(np.median(x)) if len(x) else float("nan")

    def _rms(x: np.ndarray) -> float:
        x = x[np.isfinite(x)]
        return float(np.sqrt(np.mean(x**2))) if len(x) else float("nan")

    e_quiet = _rms(e[early]) * 1000
    e_strong = _med(e[strong]) * 1000 if strong.any() else float("nan")
    ratio = e[strong] / np.maximum(d[strong], 1e-6)
    ratio_med = _med(ratio) if strong.any() else float("nan")
    e_static = _med(e[static]) * 1000 if static.any() else float("nan")
    v12_rms = _rms(v12[strong]) if strong.any() else _rms(v12)
    edt_rms = _rms(np.abs(edt[strong])) * 1000 if strong.any() else float("nan")
    sat_frac = float(np.mean(sat[strong])) if strong.any() else float(np.mean(sat))
    tf_frac = float(np.mean(tf > 0.5))

    # invalid if TF mostly missing
    if tf_frac < 0.9:
        return TuneScore(
            run_dir=str(run_dir),
            valid=False,
            score=w["w_invalid"],
            reason=f"TF dropout (ok_frac={tf_frac:.2f})",
            duration_s=duration,
            tf_ok_frac=tf_frac,
        )

    # score (nan → penalty)
    def _or(x: float, pen: float) -> float:
        return pen if (x != x) else x

    score = (
        w["w_strong_e"] * _or(e_strong, 80.0)
        + w["w_static_e"] * _or(e_static, 40.0)
        + w["w_ratio"] * _or(ratio_med, 1.5)
        + w["w_chatter"] * _or(v12_rms, 2.0)
        + w["w_edt"] * _or(edt_rms, 200.0)
        + w["w_sat"] * sat_frac
    )
    # soft penalty if no static-return segment (protocol incomplete)
    if static.sum() < 40:
        score += 15.0
        reason_extra = "incomplete return-to-origin (few Δ<10mm samples)"
    else:
        reason_extra = "ok"

    return TuneScore(
        run_dir=str(run_dir),
        valid=True,
        score=float(score),
        reason=reason_extra,
        duration_s=duration,
        e_quiet_rms_mm=e_quiet,
        e_strong_med_mm=e_strong,
        ratio_strong_med=ratio_med,
        e_static_med_mm=e_static,
        delta_max_mm=float(np.nanmax(d) * 1000),
        e_max_mm=float(np.nanmax(e) * 1000),
        v12_rms=v12_rms,
        edt_rms_mm_s=edt_rms,
        tau_sat_frac=sat_frac,
        tf_ok_frac=tf_frac,
        n_strong=int(strong.sum()),
        n_static=int(static.sum()),
    )


def append_trial_log(session_dir: Path, record: dict[str, Any]) -> None:
    session_dir.mkdir(parents=True, exist_ok=True)
    path = session_dir / "trials.jsonl"
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def load_trial_log(session_dir: Path) -> list[dict[str, Any]]:
    path = Path(session_dir) / "trials.jsonl"
    if not path.is_file():
        return []
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            out.append(json.loads(line))
    return out


def best_from_log(trials: list[dict[str, Any]]) -> Optional[ModeCParams]:
    valid = [t for t in trials if t.get("score", {}).get("valid")]
    if not valid:
        return None
    best = min(valid, key=lambda t: float(t["score"]["score"]))
    return ModeCParams.from_dict(best["params"])


def format_score_table(scores: list[tuple[ModeCParams, TuneScore]]) -> str:
    lines = [
        f"{'rank':>4} {'score':>8} {'e_str':>7} {'e_stat':>7} {'e/Δ':>6} "
        f"{'v12':>5} {'key'}",
        "-" * 72,
    ]
    ranked = sorted(scores, key=lambda x: (not x[1].valid, x[1].score))
    for i, (p, s) in enumerate(ranked, 1):
        lines.append(
            f"{i:4d} {s.score:8.2f} "
            f"{s.e_strong_med_mm:7.1f} {s.e_static_med_mm:7.1f} "
            f"{s.ratio_strong_med:6.2f} {s.v12_rms:5.2f} {p.key()}"
        )
    return "\n".join(lines)


def suggest_next(
    strategy: str,
    base: ModeCParams,
    tried: set[str],
    grid: Optional[dict[str, list[float]]] = None,
    axis_cycle: Optional[list[str]] = None,
    trial_index: int = 0,
    seed: int = 0,
) -> ModeCParams:
    """根据策略给出下一组参数。"""
    strategy = strategy.lower()
    if strategy == "grid":
        for p in expand_grid(grid=grid, base=base):
            if p.key() not in tried:
                return p
        # exhausted → random
        strategy = "random"

    if strategy == "coordinate":
        axes = axis_cycle or [
            "kp_pos",
            "osc_lambda",
            "hw_torque_lpf_alpha",
            "kp_ori",
            "kd_pos",
            "kd_ori",
        ]
        axis = axes[trial_index % len(axes)]
        nxt = coordinate_next(base, axis, grid=grid, tried=tried)
        if nxt is not None:
            return nxt
        # try other axes
        for a in axes:
            nxt = coordinate_next(base, a, grid=grid, tried=tried)
            if nxt is not None:
                return nxt
        strategy = "random"

    # random
    for k in range(1000):
        p = random_candidates(1, base=base, seed=seed + trial_index * 1000 + k)[0]
        if p.key() not in tried:
            return p
    return base.clamp()
