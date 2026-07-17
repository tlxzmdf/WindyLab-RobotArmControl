#!/usr/bin/env python3
"""Analyze hardware MIT trajectory test runs (cosine / line / circle).

Reads trajectory.csv under data/mit_traj/<stamp>_<task>/ and writes:
  - analysis.json  (per-task + summary)
  - analysis_summary.txt

Usage:
  python3 scripts/analyze_hw_mit_traj.py \\
      --runs 20260717_160251_cosine,20260717_160302_line,20260717_160314_circle
  # or auto-pick latest of each task:
  python3 scripts/analyze_hw_mit_traj.py --latest
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MIT_DIR = PROJECT_ROOT / "data" / "mit_traj"

# Rough judgment bands for EE position tracking under demo-sized motion
# (A↔B ~316 mm, circle r=80 mm) with MIT position mode.
BANDS_EE_MEAN_MM = [
    (10.0, "excellent"),
    (25.0, "good"),
    (40.0, "fair"),
    (80.0, "poor"),
]
BANDS_EE_MAX_MM = [
    (25.0, "excellent"),
    (50.0, "good"),
    (80.0, "fair"),
    (150.0, "poor"),
]
BANDS_J1_MEAN_RAD = [
    (0.03, "excellent"),
    (0.06, "good"),
    (0.10, "fair"),
    (0.20, "poor"),
]


def _grade(value: float, bands: list[tuple[float, str]]) -> str:
    for thr, name in bands:
        if value <= thr:
            return name
    return "fail"


def _load_csv(path: Path) -> dict[str, np.ndarray]:
    import csv

    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty {path}")

    def col(k: str) -> np.ndarray:
        return np.array([float(r[k]) for r in rows], dtype=float)

    data = {"t": col("t")}
    for i in range(1, 8):
        data[f"q_cmd_{i}"] = col(f"q_cmd_{i}")
        data[f"q_meas_{i}"] = col(f"q_meas_{i}")
        data[f"dq_cmd_{i}"] = col(f"dq_cmd_{i}")
        data[f"dq_meas_{i}"] = col(f"dq_meas_{i}")
    for pref in ("ee_cmd", "ee_meas"):
        for ax in ("x", "y", "z", "roll", "pitch", "yaw"):
            data[f"{pref}_{ax}"] = col(f"{pref}_{ax}")
    return data


def _pct(a: np.ndarray, p: float) -> float:
    return float(np.percentile(a, p))


def analyze_run(run_dir: Path) -> dict[str, Any]:
    csv_path = run_dir / "trajectory.csv"
    data = _load_csv(csv_path)
    t = data["t"]
    dt = np.diff(t)
    rate_hz = float(1.0 / np.median(dt)) if dt.size else float("nan")

    err_xyz = np.stack(
        [
            data["ee_cmd_x"] - data["ee_meas_x"],
            data["ee_cmd_y"] - data["ee_meas_y"],
            data["ee_cmd_z"] - data["ee_meas_z"],
        ],
        axis=1,
    )
    pos_err = np.linalg.norm(err_xyz, axis=1)
    axis_abs = np.abs(err_xyz)

    # Orientation: demo IK is position-only; cmd rpy is held at start R0.
    # Still report measured RPY travel as motion richness.
    rpy_meas = np.stack(
        [data["ee_meas_roll"], data["ee_meas_pitch"], data["ee_meas_yaw"]], axis=1
    )
    rpy_span = np.ptp(rpy_meas, axis=0)

    q_err = []
    q_cmd_span = []
    q_meas_span = []
    for i in range(1, 8):
        e = np.abs(data[f"q_cmd_{i}"] - data[f"q_meas_{i}"])
        q_err.append(e)
        q_cmd_span.append(float(np.ptp(data[f"q_cmd_{i}"])))
        q_meas_span.append(float(np.ptp(data[f"q_meas_{i}"])))
    q_err_arr = np.stack(q_err, axis=0)  # (7, N)

    # Lag proxy: argmax of cross-correlation between cmd and meas EE Y
    # (dominant demo axis). Positive lag_samples => meas lags cmd.
    cy, my = data["ee_cmd_y"], data["ee_meas_y"]
    cy0, my0 = cy - cy.mean(), my - my.mean()
    if np.std(cy0) > 1e-6 and np.std(my0) > 1e-6:
        corr = np.correlate(my0, cy0, mode="full")
        lags = np.arange(-len(cy) + 1, len(cy))
        best = int(lags[int(np.argmax(corr))])
        lag_s = best / rate_hz if rate_hz == rate_hz else float("nan")
    else:
        best, lag_s = 0, float("nan")

    tracking_ratio = []
    for i in range(7):
        cspan = q_cmd_span[i]
        mspan = q_meas_span[i]
        tracking_ratio.append(float(mspan / cspan) if cspan > 1e-4 else 1.0)

    out: dict[str, Any] = {
        "run_dir": str(run_dir),
        "task": run_dir.name.split("_", 2)[-1] if "_" in run_dir.name else run_dir.name,
        "n_samples": int(t.size),
        "duration_s": float(t[-1] - t[0]) if t.size else 0.0,
        "rate_hz_median": rate_hz,
        "ee_pos_err_mm": {
            "mean": float(np.mean(pos_err) * 1e3),
            "rms": float(np.sqrt(np.mean(pos_err**2)) * 1e3),
            "p95": _pct(pos_err, 95) * 1e3,
            "max": float(np.max(pos_err) * 1e3),
        },
        "ee_axis_abs_err_mm": {
            "x_mean": float(np.mean(axis_abs[:, 0]) * 1e3),
            "y_mean": float(np.mean(axis_abs[:, 1]) * 1e3),
            "z_mean": float(np.mean(axis_abs[:, 2]) * 1e3),
            "x_max": float(np.max(axis_abs[:, 0]) * 1e3),
            "y_max": float(np.max(axis_abs[:, 1]) * 1e3),
            "z_max": float(np.max(axis_abs[:, 2]) * 1e3),
        },
        "ee_cmd_travel_mm": {
            "x": float(np.ptp(data["ee_cmd_x"]) * 1e3),
            "y": float(np.ptp(data["ee_cmd_y"]) * 1e3),
            "z": float(np.ptp(data["ee_cmd_z"]) * 1e3),
            "path": float(
                np.sum(np.linalg.norm(np.diff(
                    np.stack(
                        [data["ee_cmd_x"], data["ee_cmd_y"], data["ee_cmd_z"]],
                        axis=1,
                    ),
                    axis=0,
                ), axis=1))
                * 1e3
            ),
        },
        "ee_meas_travel_mm": {
            "x": float(np.ptp(data["ee_meas_x"]) * 1e3),
            "y": float(np.ptp(data["ee_meas_y"]) * 1e3),
            "z": float(np.ptp(data["ee_meas_z"]) * 1e3),
        },
        "rpy_meas_span_deg": {
            "roll": float(np.degrees(rpy_span[0])),
            "pitch": float(np.degrees(rpy_span[1])),
            "yaw": float(np.degrees(rpy_span[2])),
        },
        "joint_abs_err_rad": {
            "mean": [float(np.mean(q_err_arr[i])) for i in range(7)],
            "max": [float(np.max(q_err_arr[i])) for i in range(7)],
            "rms": [float(np.sqrt(np.mean(q_err_arr[i] ** 2))) for i in range(7)],
        },
        "joint_span_rad": {
            "cmd": q_cmd_span,
            "meas": q_meas_span,
            "meas_over_cmd": tracking_ratio,
        },
        "lag_y_samples": best,
        "lag_y_s": lag_s,
        "grades": {},
    }
    out["grades"]["ee_mean"] = _grade(out["ee_pos_err_mm"]["mean"], BANDS_EE_MEAN_MM)
    out["grades"]["ee_max"] = _grade(out["ee_pos_err_mm"]["max"], BANDS_EE_MAX_MM)
    out["grades"]["j1_mean"] = _grade(
        out["joint_abs_err_rad"]["mean"][0], BANDS_J1_MEAN_RAD
    )
    # Overall: worst of the three primary grades
    order = ["excellent", "good", "fair", "poor", "fail"]
    out["grades"]["overall"] = max(
        (out["grades"]["ee_mean"], out["grades"]["ee_max"], out["grades"]["j1_mean"]),
        key=lambda g: order.index(g),
    )
    return out


def find_latest_per_task() -> list[Path]:
    latest: dict[str, Path] = {}
    for d in sorted(MIT_DIR.iterdir()):
        if not d.is_dir():
            continue
        parts = d.name.split("_")
        if len(parts) < 3:
            continue
        task = parts[-1]
        if task not in ("cosine", "line", "circle"):
            continue
        if (d / "trajectory.csv").is_file():
            latest[task] = d  # sorted names → last wins
    order = ["cosine", "line", "circle"]
    return [latest[t] for t in order if t in latest]


def summarize(results: list[dict[str, Any]]) -> dict[str, Any]:
    means = [r["ee_pos_err_mm"]["mean"] for r in results]
    maxes = [r["ee_pos_err_mm"]["max"] for r in results]
    j1 = [r["joint_abs_err_rad"]["mean"][0] for r in results]
    order = ["excellent", "good", "fair", "poor", "fail"]
    overall = max((r["grades"]["overall"] for r in results), key=lambda g: order.index(g))

    verdict_lines = []
    if overall in ("excellent", "good"):
        verdict_lines.append(
            "MIT + 真机在 student-demo 幅度下跟踪可用：末端误差厘米级，关节能跟上指令。"
        )
    elif overall == "fair":
        verdict_lines.append(
            "基本可用，但末端峰值误差偏大；适合验证链路，精细稳定仍需再调。"
        )
    else:
        verdict_lines.append(
            "跟踪偏弱或异常；优先查串口/MIT增益/指令超时，不宜当作稳定基线。"
        )

    # Relative path length coverage
    for r in results:
        cmd_y = r["ee_cmd_travel_mm"]["y"]
        meas_y = r["ee_meas_travel_mm"]["y"]
        if cmd_y > 1.0:
            cov = meas_y / cmd_y
            r["y_coverage"] = cov
            if cov < 0.7:
                verdict_lines.append(
                    f"{r['task']}: Y 行程覆盖仅 {cov:.0%}（指令 {cmd_y:.0f} mm vs 实测 {meas_y:.0f} mm）。"
                )

    return {
        "n_tasks": len(results),
        "ee_mean_mm_avg": float(np.mean(means)) if means else None,
        "ee_max_mm_worst": float(np.max(maxes)) if maxes else None,
        "j1_mean_rad_avg": float(np.mean(j1)) if j1 else None,
        "overall_grade": overall,
        "verdict": " ".join(verdict_lines),
        "grade_legend": {
            "ee_mean_mm": "≤10 excellent / ≤25 good / ≤40 fair / ≤80 poor",
            "ee_max_mm": "≤25 excellent / ≤50 good / ≤80 fair / ≤150 poor",
            "j1_mean_rad": "≤0.03 excellent / ≤0.06 good / ≤0.10 fair / ≤0.20 poor",
        },
    }


def format_text(results: list[dict[str, Any]], summary: dict[str, Any]) -> str:
    lines = [
        "=== HW MIT trajectory analysis ===",
        f"overall: {summary['overall_grade']}",
        f"avg ee_mean: {summary['ee_mean_mm_avg']:.2f} mm",
        f"worst ee_max: {summary['ee_max_mm_worst']:.2f} mm",
        f"avg |q1| err: {summary['j1_mean_rad_avg']:.4f} rad "
        f"({math.degrees(summary['j1_mean_rad_avg']):.2f} deg)",
        "",
        summary["verdict"],
        "",
    ]
    for r in results:
        e = r["ee_pos_err_mm"]
        ax = r["ee_axis_abs_err_mm"]
        lines.append(f"--- {r['task']} ({Path(r['run_dir']).name}) ---")
        lines.append(
            f"  EE err mm: mean={e['mean']:.2f} rms={e['rms']:.2f} "
            f"p95={e['p95']:.2f} max={e['max']:.2f}  [{r['grades']['overall']}]"
        )
        lines.append(
            f"  axis mean mm: X={ax['x_mean']:.1f} Y={ax['y_mean']:.1f} Z={ax['z_mean']:.1f}"
        )
        lines.append(
            f"  travel cmd/meas Y mm: "
            f"{r['ee_cmd_travel_mm']['y']:.1f} / {r['ee_meas_travel_mm']['y']:.1f}"
        )
        jm = r["joint_abs_err_rad"]["mean"]
        lines.append(
            "  joint |err| mean rad: "
            + ", ".join(f"j{i+1}={jm[i]:.3f}" for i in range(7))
        )
        lines.append(
            f"  lag(Y) ≈ {r['lag_y_s']*1000:.0f} ms" if r["lag_y_s"] == r["lag_y_s"] else "  lag(Y): n/a"
        )
        lines.append("")
    lines.append("bands: " + json.dumps(summary["grade_legend"], ensure_ascii=False))
    return "\n".join(lines)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--latest", action="store_true", help="Use latest cosine/line/circle")
    p.add_argument(
        "--runs",
        type=str,
        default="",
        help="Comma-separated run dir names under data/mit_traj/",
    )
    p.add_argument(
        "--out-dir",
        type=str,
        default="",
        help="Where to write analysis.json (default: first run's parent or mit_traj)",
    )
    args = p.parse_args()

    if args.latest or not args.runs:
        run_dirs = find_latest_per_task()
    else:
        run_dirs = [MIT_DIR / name.strip() for name in args.runs.split(",") if name.strip()]

    if not run_dirs:
        print("no runs found", file=sys.stderr)
        return 2

    results = [analyze_run(d) for d in run_dirs]
    summary = summarize(results)
    payload = {"summary": summary, "tasks": results}

    out_dir = Path(args.out_dir) if args.out_dir else MIT_DIR / "analysis_latest"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "analysis.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    )
    text = format_text(results, summary)
    (out_dir / "analysis_summary.txt").write_text(text + "\n")
    print(text)
    print(f"\nwrote {out_dir / 'analysis.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
