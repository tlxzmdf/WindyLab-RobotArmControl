#!/usr/bin/env python3
"""Phase 0 diagnostics for Mode B HW runs (aligned.csv).

Metrics cover EE tracking, actuator saturation, and sawtooth/chatter proxies.
Writes phase0_baseline.json (+ optional .txt) under the run directory.

Usage:
  python3 scripts/analyze_mode_b_phase0.py \\
      --run data/runs/20260717_172628_B_mode_b_mit_kp60kd15
  python3 scripts/analyze_mode_b_phase0.py --latest
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNS_DIR = PROJECT_ROOT / "data" / "runs"

# Limits matching stabilization_hw_mode_b.yaml (pre-Phase-1 defaults if meta missing)
DEFAULT_VMAX = [0.55, 0.75, 0.75, 0.60, 0.55, 0.55]
DEFAULT_TAU_LIM = 9.0


def _f(row: dict[str, str], *keys: str) -> float:
    for k in keys:
        v = row.get(k, "")
        if v == "" or v is None:
            continue
        try:
            x = float(v)
            if x == x:
                return x
        except ValueError:
            pass
    return float("nan")


def _load_aligned(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty {path}")
    return rows


def _col(rows: list[dict[str, str]], *keys: str) -> np.ndarray:
    return np.array([_f(r, *keys) for r in rows], dtype=float)


def _finite(a: np.ndarray) -> np.ndarray:
    return a[np.isfinite(a)]


def _pct(a: np.ndarray, p: float) -> float:
    a = _finite(a)
    return float(np.percentile(a, p)) if a.size else float("nan")


def _mean(a: np.ndarray) -> float:
    a = _finite(a)
    return float(np.mean(a)) if a.size else float("nan")


def _maxabs(a: np.ndarray) -> float:
    a = _finite(a)
    return float(np.max(np.abs(a))) if a.size else float("nan")


def _zero_cross_rate(x: np.ndarray) -> float:
    x = _finite(x)
    if x.size < 3:
        return float("nan")
    d = np.diff(x)
    if d.size < 2:
        return float("nan")
    return float(np.mean(d[:-1] * d[1:] < 0))


def _psd_band_shares(t: np.ndarray, y: np.ndarray, dt: float = 0.02) -> dict[str, float]:
    m = np.isfinite(t) & np.isfinite(y)
    if np.count_nonzero(m) < 50:
        return {"low_0_2hz": float("nan"), "mid_2_8hz": float("nan"), "high_8_20hz": float("nan")}
    tt, yy = t[m], y[m]
    tr = np.arange(tt[0], tt[-1], dt)
    if tr.size < 64:
        return {"low_0_2hz": float("nan"), "mid_2_8hz": float("nan"), "high_8_20hz": float("nan")}
    yr = np.interp(tr, tt, yy)
    yr = yr - np.mean(yr)
    F = np.abs(np.fft.rfft(yr)) ** 2
    f = np.fft.rfftfreq(len(yr), dt)

    def band(lo: float, hi: float) -> float:
        mask = (f >= lo) & (f < hi)
        return float(np.sum(F[mask]))

    tot = band(0.1, 20.0)
    if tot <= 0:
        return {"low_0_2hz": float("nan"), "mid_2_8hz": float("nan"), "high_8_20hz": float("nan")}
    return {
        "low_0_2hz": band(0.1, 2.0) / tot,
        "mid_2_8hz": band(2.0, 8.0) / tot,
        "high_8_20hz": band(8.0, 20.0) / tot,
    }


def analyze_run(
    run_dir: Path,
    vmax: list[float] | None = None,
    tau_lim: float = DEFAULT_TAU_LIM,
) -> dict[str, Any]:
    rows = _load_aligned(run_dir / "aligned.csv")
    vmax = list(vmax or DEFAULT_VMAX)
    t = _col(rows, "t_sec")
    dt = float(np.nanmedian(np.diff(_finite(t)))) if t.size > 2 else float("nan")

    e_pos = _col(rows, "world_pos_err_m") * 1000.0  # mm
    e_ori = _col(rows, "world_orient_err_rad")
    ee_x = _col(rows, "ee_x")
    ee_y = _col(rows, "ee_y")
    ee_z = _col(rows, "ee_z")
    tgt_x = _col(rows, "target_x")
    tgt_y = _col(rows, "target_y")
    tgt_z = _col(rows, "target_z")

    out: dict[str, Any] = {
        "run": run_dir.name,
        "n_samples": len(rows),
        "duration_s": float(np.nanmax(t) - np.nanmin(t)) if t.size else float("nan"),
        "dt_median_s": dt,
        "limits": {"vmax_rad_s": vmax, "tau_lim_nm": tau_lim},
        "ee": {},
        "saturation": {},
        "chatter": {},
        "joints": {},
    }

    out["ee"] = {
        "pos_err_mm": {
            "rms": float(np.sqrt(np.nanmean(e_pos**2))),
            "mean": _mean(e_pos),
            "p95": _pct(e_pos, 95),
            "max": float(np.nanmax(e_pos)),
        },
        "orient_err_rad": {
            "rms": float(np.sqrt(np.nanmean(e_ori**2))),
            "mean": _mean(e_ori),
            "p95": _pct(e_ori, 95),
            "max": float(np.nanmax(e_ori)),
        },
        "axis_err_mm": {
            "x": {
                "rms": float(np.sqrt(np.nanmean((ee_x - tgt_x) ** 2)) * 1000),
                "max": _maxabs(ee_x - tgt_x) * 1000,
            },
            "y": {
                "rms": float(np.sqrt(np.nanmean((ee_y - tgt_y) ** 2)) * 1000),
                "max": _maxabs(ee_y - tgt_y) * 1000,
            },
            "z": {
                "rms": float(np.sqrt(np.nanmean((ee_z - tgt_z) ** 2)) * 1000),
                "max": _maxabs(ee_z - tgt_z) * 1000,
            },
        },
        "pos_err_psd_share": _psd_band_shares(t, e_pos),
    }

    sat_any = np.zeros(len(rows), dtype=bool)
    for j in range(1, 7):
        lim = vmax[j - 1] if j - 1 < len(vmax) else vmax[-1]
        dq = _col(rows, f"joint{j}_dq_cmd")
        tau = _col(rows, f"joint{j}_tau_ff")
        dq_sat = np.abs(dq) >= lim * 0.99
        tau_sat = np.abs(tau) >= tau_lim * 0.99
        sat_any |= np.nan_to_num(dq_sat, nan=False) | np.nan_to_num(tau_sat, nan=False)
        out["saturation"][f"j{j}"] = {
            "dq_cmd_p95": _pct(np.abs(dq), 95),
            "dq_cmd_max": _maxabs(dq),
            "dq_sat_frac": float(np.nanmean(dq_sat)),
            "tau_ff_p95": _pct(np.abs(tau), 95),
            "tau_ff_max": _maxabs(tau),
            "tau_sat_frac": float(np.nanmean(tau_sat)),
        }

    out["saturation"]["any_j1_tau_or_dq_frac"] = float(np.mean(sat_any))
    if np.any(sat_any) and np.any(~sat_any):
        out["saturation"]["ee_err_mm_when_sat"] = _mean(e_pos[sat_any])
        out["saturation"]["ee_err_mm_when_not_sat"] = _mean(e_pos[~sat_any])

    out["chatter"] = {
        "ee_pos_err_de_zero_cross_per_sample": _zero_cross_rate(e_pos),
        "ee_pos_err_de_zero_cross_per_s": (
            _zero_cross_rate(e_pos) / dt if dt and dt > 0 else float("nan")
        ),
    }
    for j in (1, 2, 5):
        q = _col(rows, f"joint{j}_fb_pos", f"joint{j}_pos")
        if t.size > 2 and np.sum(np.isfinite(q)) > 10:
            dq = np.diff(q) / np.diff(t)
            zc = _zero_cross_rate(dq)
            out["chatter"][f"j{j}_vel_zero_cross_per_sample"] = zc
            out["chatter"][f"j{j}_vel_zero_cross_per_s"] = (
                zc / dt if dt and dt > 0 else float("nan")
            )
            out["chatter"][f"j{j}_dq_meas_p95"] = _pct(np.abs(dq), 95)

        qc = _col(rows, f"joint{j}_q_cmd")
        dqc = np.abs(np.diff(qc))
        out["joints"][f"j{j}"] = {
            "q_cmd_median_abs_step": float(np.nanmedian(dqc)),
            "q_cmd_frac_step_gt_1e-4": float(np.nanmean(dqc > 1e-4)),
            "q_cmd_frac_flat": float(np.nanmean(dqc < 1e-6)),
        }

    # Track error j1 (cmd vs fb) if both present
    for j in range(1, 7):
        qc = _col(rows, f"joint{j}_q_cmd")
        qf = _col(rows, f"joint{j}_fb_pos", f"joint{j}_pos")
        err = qc - qf
        key = f"j{j}"
        if key not in out["joints"]:
            out["joints"][key] = {}
        out["joints"][key]["track_err_rms_rad"] = float(np.sqrt(np.nanmean(err**2)))
        out["joints"][key]["track_err_max_rad"] = _maxabs(err)

    return out


def _format_txt(report: dict[str, Any]) -> str:
    ee = report["ee"]
    sat = report["saturation"]
    ch = report["chatter"]
    lines = [
        f"Phase 0 baseline: {report['run']}",
        f"  duration={report['duration_s']:.1f}s  n={report['n_samples']}  dt≈{report['dt_median_s']*1000:.1f}ms",
        f"EE pos err mm: rms={ee['pos_err_mm']['rms']:.1f}  mean={ee['pos_err_mm']['mean']:.1f}  "
        f"p95={ee['pos_err_mm']['p95']:.1f}  max={ee['pos_err_mm']['max']:.1f}",
        f"EE orient err rad: rms={ee['orient_err_rad']['rms']:.3f}  max={ee['orient_err_rad']['max']:.3f}",
        f"Axis RMS mm: x={ee['axis_err_mm']['x']['rms']:.1f}  y={ee['axis_err_mm']['y']['rms']:.1f}  "
        f"z={ee['axis_err_mm']['z']['rms']:.1f}",
        f"EE err PSD share: low={ee['pos_err_psd_share']['low_0_2hz']:.2f}  "
        f"mid={ee['pos_err_psd_share']['mid_2_8hz']:.2f}  high={ee['pos_err_psd_share']['high_8_20hz']:.2f}",
        f"Sat any (j1 τ|dq proxy) frac={sat.get('any_j1_tau_or_dq_frac', float('nan')):.3f}",
    ]
    for j in range(1, 4):
        s = sat[f"j{j}"]
        lines.append(
            f"  j{j}: dq_sat={s['dq_sat_frac']:.3f}  tau_sat={s['tau_sat_frac']:.3f}  "
            f"|dq|p95={s['dq_cmd_p95']:.3f}  |τ|p95={s['tau_ff_p95']:.3f}"
        )
    lines.append(
        f"Chatter: ee de zc/s={ch.get('ee_pos_err_de_zero_cross_per_s', float('nan')):.1f}  "
        f"j1 vel zc/s={ch.get('j1_vel_zero_cross_per_s', float('nan')):.1f}  "
        f"j5 vel zc/s={ch.get('j5_vel_zero_cross_per_s', float('nan')):.1f}"
    )
    return "\n".join(lines) + "\n"


def _latest_mode_b_run() -> Path:
    cands = sorted(
        [p for p in RUNS_DIR.glob("*_B_*") if (p / "aligned.csv").is_file()],
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not cands:
        cands = sorted(
            [p for p in RUNS_DIR.iterdir() if p.is_dir() and (p / "aligned.csv").is_file()],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
    if not cands:
        raise FileNotFoundError(f"no runs with aligned.csv under {RUNS_DIR}")
    return cands[0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=str, default="", help="Run dir or name under data/runs/")
    ap.add_argument("--latest", action="store_true", help="Use latest Mode B-like run")
    ap.add_argument("--tau-lim", type=float, default=DEFAULT_TAU_LIM)
    ap.add_argument(
        "--vmax",
        type=str,
        default="",
        help="Comma-separated per-joint vmax (default Mode B pre-P1)",
    )
    args = ap.parse_args()

    if args.latest or not args.run:
        run_dir = _latest_mode_b_run()
    else:
        p = Path(args.run)
        run_dir = p if p.is_dir() else RUNS_DIR / args.run
    if not (run_dir / "aligned.csv").is_file():
        raise SystemExit(f"missing aligned.csv in {run_dir}")

    vmax = DEFAULT_VMAX
    if args.vmax:
        vmax = [float(x) for x in args.vmax.split(",")]

    report = analyze_run(run_dir, vmax=vmax, tau_lim=args.tau_lim)
    report["phase"] = 0
    report["label"] = "pre_phase1_baseline"

    json_path = run_dir / "phase0_baseline.json"
    txt_path = run_dir / "phase0_baseline.txt"
    json_path.write_text(json.dumps(report, indent=2) + "\n")
    txt = _format_txt(report)
    txt_path.write_text(txt)
    print(txt)
    print(f"Wrote {json_path}")
    print(f"Wrote {txt_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
