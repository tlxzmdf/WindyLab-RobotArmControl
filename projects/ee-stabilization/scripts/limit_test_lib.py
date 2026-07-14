#!/usr/bin/env python3
"""Shared utilities for EE stabilization limit-test sweeps."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
WS = ARM_ROOT / "windylab_ws"
LAUNCH_FILE = PROJECT_ROOT / "launch" / "limit_test_headless.launch.py"
REPORT_ROOT = PROJECT_ROOT / "reports" / "limit_test"

WARMUP_SEC = 3.0
RECORD_SEC = 20.0
EE_FRAME = "link6"
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
MOUNT_JOINTS = ["mount_tx", "mount_ty", "mount_tz", "mount_rx", "mount_ry", "mount_rz"]

NOMINAL_RADIUS = 0.35
NOMINAL_ORIENT_AMP = 0.32
NOMINAL_TIME_CONSTANT = 2.0
NOMINAL_AMPLITUDE_SCALE = 0.90

DEFAULT_BASELINE_POS_RMS_MM = 0.2495
DEFAULT_BASELINE_ORIENT_RMS_DEG = 0.0268

MODES = {
    "A": {
        "label": "模式 A: IK + 纯运动学稳定",
        "use_ik_joint_control": True,
        "kinematic_stabilization": True,
    },
    "B": {
        "label": "模式 B: IK + 关节空间计算力矩控制",
        "use_ik_joint_control": True,
        "kinematic_stabilization": False,
    },
    "C": {
        "label": "模式 C: 任务空间操作空间控制",
        "use_ik_joint_control": False,
        "kinematic_stabilization": False,
    },
    "D": {
        "label": "模式 D: sat 速度规划 + OSC + ESO",
        "stabilization_mode": "D",
        "use_ik_joint_control": False,
        "kinematic_stabilization": False,
    },
}

FAIL_ABS_POS_RMS_MM = 5.0
FAIL_ABS_ORIENT_RMS_DEG = 2.0
FAIL_ABS_POS_P95_MM = 10.0
FAIL_REL_POS_RMS_FACTOR = 10.0

_CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if Path(_CJK_FONT).exists():
    fm.fontManager.addfont(_CJK_FONT)
    plt.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False


@dataclass(frozen=True)
class DisturbanceCase:
    study: str
    mode: str
    radius: float
    orient_amp: float
    time_constant: float
    amplitude_scale: float = NOMINAL_AMPLITUDE_SCALE
    seed: int = 42
    scale: Optional[float] = None

    @property
    def effective_radius(self) -> float:
        return self.radius * self.amplitude_scale

    @property
    def f_seg_hz(self) -> float:
        return 1.0 / max(self.time_constant, 0.05)

    @property
    def v_peak_mps(self) -> float:
        return 1.5 * self.effective_radius / max(self.time_constant, 0.05)

    @property
    def w_peak_rps(self) -> float:
        return 1.5 * self.orient_amp / max(self.time_constant, 0.05)

    @property
    def run_id(self) -> str:
        return (
            f"{self.mode}_{self.radius:.3f}_{self.orient_amp:.3f}_"
            f"{self.time_constant:.3f}_{self.seed}"
        )

    def to_dict(self) -> dict:
        d = asdict(self)
        d["effective_radius"] = self.effective_radius
        d["f_seg_hz"] = self.f_seg_hz
        d["v_peak_mps"] = self.v_peak_mps
        d["w_peak_rps"] = self.w_peak_rps
        d["run_id"] = self.run_id
        return d


@dataclass
class Sample:
    t: float
    pos_err_m: float
    orient_err_rad: float
    base_pos_err_m: float
    base_orient_err_rad: float
    ik_solve_us: float
    mount_pos: np.ndarray
    mount_vel: np.ndarray
    q_act: np.ndarray
    q_cmd: np.ndarray


@dataclass
class RunRecord:
    case: DisturbanceCase
    samples: list[Sample] = field(default_factory=list)
    target_locked: bool = False
    launch_returncode: Optional[int] = None


def _setup_matplotlib() -> None:
    plt.rcParams.update(
        {
            "font.size": 12,
            "axes.titlesize": 13,
            "axes.labelsize": 12,
            "legend.fontsize": 10,
        }
    )


def save_record_npz(record: RunRecord, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    samples = record.samples
    payload = {
        "meta_json": json.dumps(record.case.to_dict(), ensure_ascii=False),
        "t": np.array([s.t for s in samples]),
        "pos_err_m": np.array([s.pos_err_m for s in samples]),
        "orient_err_rad": np.array([s.orient_err_rad for s in samples]),
        "base_pos_err_m": np.array([s.base_pos_err_m for s in samples]),
        "base_orient_err_rad": np.array([s.base_orient_err_rad for s in samples]),
        "ik_solve_us": np.array([s.ik_solve_us for s in samples]),
        "mount_pos": np.array([s.mount_pos for s in samples]),
        "mount_vel": np.array([s.mount_vel for s in samples]),
        "q_act": np.array([s.q_act for s in samples]),
        "q_cmd": np.array([s.q_cmd for s in samples]),
        "target_locked": np.array([record.target_locked]),
    }
    np.savez_compressed(path, **payload)


def load_record_npz(path: Path) -> RunRecord:
    data = np.load(path, allow_pickle=True)
    meta = json.loads(str(data["meta_json"]))
    case = DisturbanceCase(
        study=meta["study"],
        mode=meta["mode"],
        radius=float(meta["radius"]),
        orient_amp=float(meta["orient_amp"]),
        time_constant=float(meta["time_constant"]),
        amplitude_scale=float(meta.get("amplitude_scale", NOMINAL_AMPLITUDE_SCALE)),
        seed=int(meta.get("seed", 42)),
        scale=meta.get("scale"),
    )
    n = len(data["t"])
    samples = []
    for i in range(n):
        samples.append(
            Sample(
                t=float(data["t"][i]),
                pos_err_m=float(data["pos_err_m"][i]),
                orient_err_rad=float(data["orient_err_rad"][i]),
                base_pos_err_m=float(data["base_pos_err_m"][i]),
                base_orient_err_rad=float(data["base_orient_err_rad"][i]),
                ik_solve_us=float(data["ik_solve_us"][i]),
                mount_pos=data["mount_pos"][i],
                mount_vel=data["mount_vel"][i],
                q_act=data["q_act"][i],
                q_cmd=data["q_cmd"][i],
            )
        )
    return RunRecord(case=case, samples=samples, target_locked=bool(data["target_locked"][0]))


def _stat(values: np.ndarray) -> dict[str, float]:
    if values.size == 0:
        return {"mean": math.nan, "max": math.nan, "rms": math.nan, "p95": math.nan}
    return {
        "mean": float(values.mean()),
        "max": float(values.max()),
        "rms": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
    }


def _detect_monotonic_divergence(pos_err_m: np.ndarray, times: np.ndarray, window_s: float = 3.0) -> bool:
    if len(times) < 20:
        return False
    dt = float(np.median(np.diff(times)))
    if dt <= 0:
        return False
    win = max(int(window_s / dt), 5)
    for i in range(0, len(times) - win, max(win // 2, 1)):
        seg = pos_err_m[i : i + win]
        if seg[-1] > 5.0 * max(seg[0], 1e-6) and seg[-1] > 0.005:
            if np.all(np.diff(seg) >= -1e-5):
                return True
    return False


def compute_metrics(record: RunRecord) -> dict:
    samples = record.samples
    case = record.case
    times = np.array([s.t for s in samples])
    pos_err_mm = np.array([s.pos_err_m for s in samples]) * 1000.0
    orient_err_deg = np.degrees(np.array([s.orient_err_rad for s in samples]))
    q_err = np.linalg.norm(
        np.array([s.q_act for s in samples]) - np.array([s.q_cmd for s in samples]),
        axis=1,
    )
    ik_us = np.array([s.ik_solve_us for s in samples])
    mount_pos = np.array([s.mount_pos for s in samples])
    mount_vel = np.array([s.mount_vel for s in samples])

    mount_range = float(np.linalg.norm(mount_pos.max(axis=0) - mount_pos.min(axis=0)))
    expected_range = 2.0 * case.effective_radius
    reject_heuristic = (
        expected_range > 0.05
        and mount_range < 0.2 * expected_range
        and case.orient_amp < 0.05
    )

    mount_speed = np.linalg.norm(mount_vel[:, :3], axis=1)
    mount_omega = np.linalg.norm(mount_vel[:, 3:], axis=1)

    f_char = math.nan
    if len(times) > 64:
        dt = float(np.median(np.diff(times)))
        if dt > 0:
            fs = 1.0 / dt
            nperseg = min(256, len(times) // 2)
            freqs, psd = welch(mount_pos[:, 0] - mount_pos[:, 0].mean(), fs=fs, nperseg=nperseg)
            if len(psd) > 1:
                f_char = float(freqs[1 + int(np.argmax(psd[1:]))])

    return {
        "run_id": case.run_id,
        "study": case.study,
        "mode": case.mode,
        "radius": case.radius,
        "orient_amp": case.orient_amp,
        "time_constant": case.time_constant,
        "amplitude_scale": case.amplitude_scale,
        "scale": case.scale,
        "seed": case.seed,
        "f_seg_hz": case.f_seg_hz,
        "v_peak_mps": case.v_peak_mps,
        "w_peak_rps": case.w_peak_rps,
        "samples": len(samples),
        "target_locked": record.target_locked,
        "position_mm": _stat(pos_err_mm),
        "orientation_deg": _stat(orient_err_deg),
        "joint_cmd_rad": _stat(q_err),
        "ik_solve_us_mean": float(ik_us.mean()) if ik_us.size else math.nan,
        "ik_solve_us_max": float(ik_us.max()) if ik_us.size else math.nan,
        "mount_range_m": mount_range,
        "mount_speed_peak_mps": float(mount_speed.max()) if mount_speed.size else math.nan,
        "mount_omega_peak_rps": float(mount_omega.max()) if mount_omega.size else math.nan,
        "waypoint_reject_heuristic": reject_heuristic,
        "f_char_hz_measured": f_char,
        "monotonic_divergence": _detect_monotonic_divergence(
            np.array([s.pos_err_m for s in samples]), times
        ),
    }


def evaluate_failure(
    metrics: dict,
    baseline_pos_rms_mm: float = DEFAULT_BASELINE_POS_RMS_MM,
) -> dict:
    pos = metrics["position_mm"]
    orient = metrics["orientation_deg"]
    codes: list[str] = []
    level = "OK"

    if metrics.get("waypoint_reject_heuristic"):
        codes.append("H1")
    if metrics.get("monotonic_divergence"):
        codes.append("H4")
    if pos["rms"] > FAIL_ABS_POS_RMS_MM:
        codes.append("T1")
    if orient["rms"] > FAIL_ABS_ORIENT_RMS_DEG:
        codes.append("T2")
    if pos["p95"] > FAIL_ABS_POS_P95_MM:
        codes.append("T3")
    if pos["rms"] > FAIL_REL_POS_RMS_FACTOR * baseline_pos_rms_mm:
        codes.append("T4")
    if pos["rms"] > 3.0 * baseline_pos_rms_mm:
        codes.append("M1")

    hard = any(c in codes for c in ("H1", "H4"))
    task = any(c in codes for c in ("T1", "T2", "T3", "T4"))
    if hard:
        level = "HARD"
    elif task:
        level = "FAIL"
    elif "M1" in codes:
        level = "MARGINAL"

    return {"fail_level": level, "fail_codes": codes}


def flatten_metrics_row(metrics: dict, failure: dict) -> dict:
    return {
        "run_id": metrics["run_id"],
        "study": metrics["study"],
        "mode": metrics["mode"],
        "radius": metrics["radius"],
        "orient_amp": metrics["orient_amp"],
        "time_constant": metrics["time_constant"],
        "amplitude_scale": metrics["amplitude_scale"],
        "scale": metrics.get("scale"),
        "seed": metrics["seed"],
        "f_seg_hz": metrics["f_seg_hz"],
        "v_peak_mps": metrics["v_peak_mps"],
        "w_peak_rps": metrics["w_peak_rps"],
        "samples": metrics["samples"],
        "pos_mean_mm": metrics["position_mm"]["mean"],
        "pos_max_mm": metrics["position_mm"]["max"],
        "pos_rms_mm": metrics["position_mm"]["rms"],
        "pos_p95_mm": metrics["position_mm"]["p95"],
        "orient_mean_deg": metrics["orientation_deg"]["mean"],
        "orient_max_deg": metrics["orientation_deg"]["max"],
        "orient_rms_deg": metrics["orientation_deg"]["rms"],
        "orient_p95_deg": metrics["orientation_deg"]["p95"],
        "joint_cmd_mean_rad": metrics["joint_cmd_rad"]["mean"],
        "mount_range_m": metrics["mount_range_m"],
        "mount_speed_peak_mps": metrics["mount_speed_peak_mps"],
        "waypoint_reject_heuristic": metrics["waypoint_reject_heuristic"],
        "f_char_hz_measured": metrics["f_char_hz_measured"],
        "fail_level": failure["fail_level"],
        "fail_codes": ",".join(failure["fail_codes"]),
    }


def metrics_json_path(npz_path: Path) -> Path:
    return npz_path.with_suffix(".json")


def save_metrics_json(metrics: dict, failure: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"metrics": metrics, "failure": failure, "row": flatten_metrics_row(metrics, failure)}
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def load_metrics_row(json_path: Path) -> dict:
    data = json.loads(json_path.read_text(encoding="utf-8"))
    return data["row"]


CSV_FIELDS = [
    "run_id", "study", "mode", "radius", "orient_amp", "time_constant", "amplitude_scale",
    "scale", "seed", "f_seg_hz", "v_peak_mps", "w_peak_rps", "samples",
    "pos_mean_mm", "pos_max_mm", "pos_rms_mm", "pos_p95_mm",
    "orient_mean_deg", "orient_max_deg", "orient_rms_deg", "orient_p95_deg",
    "joint_cmd_mean_rad", "mount_range_m", "mount_speed_peak_mps",
    "waypoint_reject_heuristic", "f_char_hz_measured", "fail_level", "fail_codes",
]


def append_csv_row(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def build_phase_cases(phase: str, modes: Iterable[str]) -> list[DisturbanceCase]:
    mode_list = list(modes)
    cases: list[DisturbanceCase] = []

    if phase in ("calibration", "all", "quick"):
        tc_values = [2.0] if phase == "quick" else [8.0, 4.0, 2.0, 1.0, 0.5, 0.25]
        for tc in tc_values:
            cases.append(
                DisturbanceCase(
                    study="calibration",
                    mode="A",
                    radius=0.20,
                    orient_amp=0.0,
                    time_constant=tc,
                )
            )

    if phase in ("amplitude", "all", "quick"):
        ap_list = [0.20, 0.35] if phase == "quick" else [
            0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50,
        ]
        ar_list = [0.20, 0.32] if phase == "quick" else [
            0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50,
        ]
        scale_list = [0.5, 1.0] if phase == "quick" else [
            0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2,
        ]
        for mode in mode_list:
            for ap in ap_list:
                cases.append(
                    DisturbanceCase(
                        study="S1_translation",
                        mode=mode,
                        radius=ap,
                        orient_amp=0.0,
                        time_constant=NOMINAL_TIME_CONSTANT,
                    )
                )
            for ar in ar_list:
                cases.append(
                    DisturbanceCase(
                        study="S2_rotation",
                        mode=mode,
                        radius=0.05,
                        orient_amp=ar,
                        time_constant=NOMINAL_TIME_CONSTANT,
                    )
                )
            for scale in scale_list:
                cases.append(
                    DisturbanceCase(
                        study="S3_scaled",
                        mode=mode,
                        radius=NOMINAL_RADIUS * scale,
                        orient_amp=NOMINAL_ORIENT_AMP * scale,
                        time_constant=NOMINAL_TIME_CONSTANT,
                        scale=scale,
                    )
                )

    if phase in ("frequency", "all", "quick"):
        tc_list = [2.0, 1.0] if phase == "quick" else [8.0, 4.0, 2.0, 1.0, 0.5, 0.25]
        for mode in mode_list:
            for tc in tc_list:
                cases.append(
                    DisturbanceCase(
                        study="frequency",
                        mode=mode,
                        radius=NOMINAL_RADIUS,
                        orient_amp=NOMINAL_ORIENT_AMP,
                        time_constant=tc,
                        scale=1.0,
                    )
                )

    if phase in ("grid", "all"):
        for mode in mode_list:
            for scale in [0.6, 0.8, 1.0, 1.2, 1.4]:
                for tc in [1.0, 1.5, 2.0, 2.5, 3.0]:
                    cases.append(
                        DisturbanceCase(
                            study="grid_2d",
                            mode=mode,
                            radius=NOMINAL_RADIUS * scale,
                            orient_amp=NOMINAL_ORIENT_AMP * scale,
                            time_constant=tc,
                            scale=scale,
                        )
                    )

    return cases


def dedupe_cases(cases: list[DisturbanceCase]) -> list[DisturbanceCase]:
    seen: set[tuple] = set()
    out: list[DisturbanceCase] = []
    for case in cases:
        key = (
            case.study,
            case.mode,
            round(case.radius, 6),
            round(case.orient_amp, 6),
            round(case.time_constant, 6),
            case.seed,
        )
        if key in seen:
            continue
        seen.add(key)
        out.append(case)
    return out


def plot_study_curves(rows: list[dict], study: str, x_key: str, x_label: str, out_path: Path) -> None:
    subset = [r for r in rows if r["study"] == study]
    if not subset:
        return
    _setup_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    colors = {"A": "C2", "B": "C0", "C": "C1"}
    for mode in ("A", "B", "C"):
        mode_rows = sorted(
            [r for r in subset if r["mode"] == mode],
            key=lambda r: float(r[x_key]) if r.get(x_key) is not None else 0.0,
        )
        if not mode_rows:
            continue
        xs = [float(r[x_key]) for r in mode_rows]
        axes[0].plot(xs, [float(r["pos_rms_mm"]) for r in mode_rows], "o-", color=colors[mode], label=f"模式 {mode}")
        axes[1].plot(xs, [float(r["orient_rms_deg"]) for r in mode_rows], "o-", color=colors[mode], label=f"模式 {mode}")

    axes[0].axhline(FAIL_ABS_POS_RMS_MM, color="0.5", ls="--", lw=1.0)
    axes[1].axhline(FAIL_ABS_ORIENT_RMS_DEG, color="0.5", ls="--", lw=1.0)
    axes[0].set_xlabel(x_label)
    axes[0].set_ylabel("位置 RMS (mm)")
    axes[1].set_xlabel(x_label)
    axes[1].set_ylabel("姿态 RMS (°)")
    axes[0].set_title(f"{study} — 位置")
    axes[1].set_title(f"{study} — 姿态")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_grid_heatmap(rows: list[dict], mode: str, out_path: Path) -> None:
    subset = [
        r for r in rows
        if r["study"] == "grid_2d" and r["mode"] == mode and r.get("scale") is not None
    ]
    if len(subset) < 4:
        return
    _setup_matplotlib()
    scales = sorted({float(r["scale"]) for r in subset})
    tcs = sorted({float(r["time_constant"]) for r in subset})
    grid = np.full((len(tcs), len(scales)), np.nan)
    for r in subset:
        i = tcs.index(float(r["time_constant"]))
        j = scales.index(float(r["scale"]))
        grid[i, j] = float(r["pos_rms_mm"])

    fig, ax = plt.subplots(figsize=(7, 5))
    im = ax.imshow(grid, aspect="auto", origin="lower", cmap="YlOrRd")
    ax.set_xticks(range(len(scales)))
    ax.set_xticklabels([f"{s:.1f}" for s in scales])
    ax.set_yticks(range(len(tcs)))
    ax.set_yticklabels([f"{t:.1f}" for t in tcs])
    ax.set_xlabel("幅度 scale")
    ax.set_ylabel("T_c (s)")
    ax.set_title(f"模式 {mode} — 位置 RMS (mm)")
    fig.colorbar(im, ax=ax, label="mm")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_calibration(rows: list[dict], out_path: Path) -> None:
    subset = sorted([r for r in rows if r["study"] == "calibration"], key=lambda r: float(r["time_constant"]))
    if not subset:
        return
    _setup_matplotlib()
    fig, ax = plt.subplots(figsize=(8, 4))
    tc = [float(r["time_constant"]) for r in subset]
    f_seg = [float(r["f_seg_hz"]) for r in subset]
    f_meas = [float(r["f_char_hz_measured"]) for r in subset]
    ax.plot(tc, f_seg, "o-", label="f_seg = 1/T_c")
    if all(not math.isnan(x) for x in f_meas):
        ax.plot(tc, f_meas, "s--", label="mount 实测主频")
    ax.set_xlabel("T_c (s)")
    ax.set_ylabel("频率 (Hz)")
    ax.set_title("Phase 0 频率标定")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def write_summary_report(rows: list[dict], out_path: Path, baseline_pos_rms_mm: float) -> None:
    lines = [
        "# 机头稳定扰动极限测试报告",
        "",
        f"- 判据: 位置 RMS > {FAIL_ABS_POS_RMS_MM} mm (T1), "
        f"姿态 RMS > {FAIL_ABS_ORIENT_RMS_DEG}° (T2), "
        f"pos RMS > {FAIL_REL_POS_RMS_FACTOR}× baseline ({baseline_pos_rms_mm:.3f} mm) (T4)",
        f"- 记录窗口: 预热 {WARMUP_SEC:.0f}s + 记录 {RECORD_SEC:.0f}s",
        f"- 总 run 数: {len(rows)}",
        "",
        "## 各 study 失效统计",
        "",
        "| study | mode | runs | FAIL/HARD | pos RMS max (mm) | orient RMS max (°) |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for study in sorted({r["study"] for r in rows}):
        for mode in ("A", "B", "C"):
            sub = [r for r in rows if r["study"] == study and r["mode"] == mode]
            if not sub:
                continue
            fail_n = sum(1 for r in sub if r["fail_level"] in ("FAIL", "HARD"))
            pos_max = max(float(r["pos_rms_mm"]) for r in sub)
            orient_max = max(float(r["orient_rms_deg"]) for r in sub)
            lines.append(
                f"| {study} | {mode} | {len(sub)} | {fail_n} | {pos_max:.3f} | {orient_max:.3f} |"
            )
    lines += [
        "",
        "完整数据见 `summary/all_results.csv`。",
    ]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
