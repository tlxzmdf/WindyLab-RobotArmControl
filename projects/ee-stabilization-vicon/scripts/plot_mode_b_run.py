#!/usr/bin/env python3
"""Plot Mode B / failure-analysis HW runs from aligned.csv (gap-free).

Resamples all series onto a uniform time grid with linear interpolation so
curves never show broken segments from TF/cmd dropouts.

Outputs under the run directory (default):
  overview_delta_err.png
  plots_lon5s/{ee_pose_6panel,joints_cmd_fb,delta_vs_err}.png
  plots_lat5s/...
  plots_peak5s/...

Usage:
  python3 scripts/plot_mode_b_run.py --run data/runs/20260717_185428_B_outerfix_1min
  python3 scripts/plot_mode_b_run.py --latest
  python3 scripts/plot_mode_b_run.py --latest --window-sec 5 --title-prefix OuterFix
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNS_DIR = PROJECT_ROOT / "data" / "runs"

_CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


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


def _setup_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt

    if Path(_CJK_FONT).exists():
        fm.fontManager.addfont(_CJK_FONT)
        plt.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def resolve_run(run_arg: str, latest: bool) -> Path:
    if latest or not run_arg:
        cands = sorted(
            [p for p in RUNS_DIR.iterdir() if p.is_dir() and (p / "aligned.csv").is_file()],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not cands:
            raise FileNotFoundError(f"no runs with aligned.csv under {RUNS_DIR}")
        return cands[0]
    p = Path(run_arg)
    if not p.is_absolute():
        p = (PROJECT_ROOT / p).resolve() if not p.exists() else p.resolve()
        if not p.exists():
            p = (RUNS_DIR / run_arg).resolve()
    if not (p / "aligned.csv").is_file():
        raise FileNotFoundError(f"missing aligned.csv in {p}")
    return p


def load_uniform_grid(
    run_dir: Path, dt_force: float | None = None
) -> dict[str, Any]:
    """Load aligned.csv and resample onto a uniform relative-time grid."""
    with (run_dir / "aligned.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty aligned.csv in {run_dir}")

    t_raw = np.array([_f(r, "t_sec") for r in rows], dtype=float)
    m_t = np.isfinite(t_raw)
    t_raw = t_raw[m_t]
    rows = [rows[i] for i in np.flatnonzero(m_t)]
    if t_raw.size < 3:
        raise ValueError("too few valid t_sec samples")

    t0 = float(t_raw[0])
    t_rel = t_raw - t0
    dt = float(dt_force) if dt_force and dt_force > 0 else float(np.median(np.diff(t_rel)))
    if not np.isfinite(dt) or dt <= 0:
        dt = 0.02
    t_u = np.arange(0.0, float(t_rel[-1]) + 0.5 * dt, dt)

    def series(*keys: str) -> np.ndarray:
        y = np.array([_f(r, *keys) for r in rows], dtype=float)
        good = np.isfinite(y) & np.isfinite(t_rel)
        if np.count_nonzero(good) < 2:
            return np.full_like(t_u, np.nan)
        order = np.argsort(t_rel[good])
        tt = t_rel[good][order]
        yy = y[good][order]
        _, uidx = np.unique(tt, return_index=True)
        return np.interp(t_u, tt[uidx], yy[uidx])

    bx = series("tf_base_x")
    by = series("tf_base_y")
    bz = series("tf_base_z")
    d = np.sqrt((bx - bx[0]) ** 2 + (by - by[0]) ** 2 + (bz - bz[0]) ** 2) * 1000.0
    dx = (bx - bx[0]) * 1000.0
    dy = (by - by[0]) * 1000.0

    # Prefer recorded relative Δ (Vicon latch); fall back to TF-from-t0.
    delta_x_mm = series("delta_x") * 1000.0
    delta_y_mm = series("delta_y") * 1000.0
    delta_z_mm = series("delta_z") * 1000.0
    if not np.any(np.isfinite(delta_x_mm)):
        delta_x_mm = dx
    if not np.any(np.isfinite(delta_y_mm)):
        delta_y_mm = dy
    if not np.any(np.isfinite(delta_z_mm)):
        delta_z_mm = (bz - bz[0]) * 1000.0
    delta_roll = series("delta_roll")
    delta_pitch = series("delta_pitch")
    delta_yaw = series("delta_yaw")
    if not np.any(np.isfinite(delta_roll)):
        delta_roll = series("tf_base_roll") - float(
            np.nanmean(series("tf_base_roll")[: max(1, int(1.0 / dt))])
        )
    if not np.any(np.isfinite(delta_pitch)):
        delta_pitch = series("tf_base_pitch") - float(
            np.nanmean(series("tf_base_pitch")[: max(1, int(1.0 / dt))])
        )
    if not np.any(np.isfinite(delta_yaw)):
        delta_yaw = series("tf_base_yaw") - float(
            np.nanmean(series("tf_base_yaw")[: max(1, int(1.0 / dt))])
        )

    early = t_u < min(5.0, float(t_u[-1]) * 0.2 + 1e-9)
    if not np.any(early):
        early = slice(0, min(50, len(t_u)))

    data: dict[str, Any] = {
        "t": t_u,
        "dt": dt,
        "d_mm": d,
        "dx_mm": dx,
        "dy_mm": dy,
        "delta_x_mm": delta_x_mm,
        "delta_y_mm": delta_y_mm,
        "delta_z_mm": delta_z_mm,
        "delta_roll": delta_roll,
        "delta_pitch": delta_pitch,
        "delta_yaw": delta_yaw,
        "e_mm": series("world_pos_err_m") * 1000.0,
        "ori_rad": series("world_orient_err_rad"),
        "ee_x": series("ee_x"),
        "ee_y": series("ee_y"),
        "ee_z": series("ee_z"),
        "target_x": series("target_x"),
        "target_y": series("target_y"),
        "target_z": series("target_z"),
        "ee_roll": series("tf_ee_roll"),
        "ee_pitch": series("tf_ee_pitch"),
        "ee_yaw": series("tf_ee_yaw"),
        "des_roll": float(np.nanmean(series("tf_ee_roll")[early])),
        "des_pitch": float(np.nanmean(series("tf_ee_pitch")[early])),
        "des_yaw": float(np.nanmean(series("tf_ee_yaw")[early])),
        "q_cmd": {},
        "q_fb": {},
    }
    for j in range(1, 8):
        data["q_cmd"][j] = series(f"joint{j}_q_cmd")
        data["q_fb"][j] = series(f"joint{j}_fb_pos", f"joint{j}_pos")
    return data


def _best_window(score: np.ndarray, win: int) -> tuple[int, int]:
    score = np.where(np.isfinite(score), score, -1e9)
    best_i, best_s = 0, -1e9
    step = max(1, win // 10)
    for i in range(0, max(1, len(score) - win), step):
        s = float(np.mean(score[i : i + win]))
        if s > best_s:
            best_s, best_i = s, i
    return best_i, best_i + win


def _best_active_window(
    score: np.ndarray,
    *,
    dt: float,
    max_sec: float = 5.0,
    min_sec: float = 1.0,
    edge_frac: float = 0.12,
) -> tuple[int, int]:
    """Pick strongest ≤max_sec window by activity, then trim quiet edges (≥min_sec)."""
    score = np.where(np.isfinite(score), score, 0.0)
    n = len(score)
    if n < 2:
        return 0, n
    max_win = min(n, max(2, int(round(max_sec / max(dt, 1e-6)))))
    min_win = min(max_win, max(2, int(round(min_sec / max(dt, 1e-6)))))
    i0, i1 = _best_window(score, max_win)
    seg = score[i0:i1]
    thr = float(np.max(seg)) * edge_frac
    # trim leading/trailing quiet samples
    left = 0
    while left < len(seg) - min_win and seg[left] < thr:
        left += 1
    right = len(seg)
    while right - left > min_win and seg[right - 1] < thr:
        right -= 1
    return i0 + left, i0 + right


def _axis_dominance_score(primary: np.ndarray, *others: np.ndarray) -> np.ndarray:
    """Score by motion activity (|d/dt|), not absolute bias — avoids flat large offsets."""
    p = np.asarray(primary, dtype=float)
    fill = float(np.nanmedian(p)) if np.any(np.isfinite(p)) else 0.0
    gp = np.abs(np.gradient(np.where(np.isfinite(p), p, fill)))
    go = np.zeros_like(gp)
    for a in others:
        aa = np.asarray(a, dtype=float)
        af = float(np.nanmedian(aa)) if np.any(np.isfinite(aa)) else 0.0
        aa = np.where(np.isfinite(aa), aa, af)
        go = go + np.abs(np.gradient(aa))
    return gp - 0.35 * go


def plot_axis_disturbance(
    data: dict[str, Any],
    i0: int,
    i1: int,
    out_path: Path,
    title: str,
    primary: np.ndarray,
    primary_label: str,
    primary_unit: str,
) -> None:
    plt = _setup_matplotlib()
    tt = data["t"][i0:i1] - data["t"][i0]
    fig, ax1 = plt.subplots(figsize=(10, 3.6))
    ax1.plot(tt, primary[i0:i1], "C0-", lw=1.2, label=primary_label)
    ax1.set_ylabel(f"{primary_label} ({primary_unit})", color="C0")
    ax1.tick_params(axis="y", labelcolor="C0")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(tt, data["e_mm"][i0:i1], "C1-", lw=1.0, label="EE err mm")
    ax2.set_ylabel("EE err (mm)", color="C1")
    ax2.tick_params(axis="y", labelcolor="C1")
    ax1.set_xlabel("t (s)")
    ax1.set_title(title)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="best", fontsize=8)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_axis_overview(data: dict[str, Any], out_path: Path, title: str) -> None:
    plt = _setup_matplotlib()
    t = data["t"]
    fig, axs = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(title)
    specs = [
        (axs[0], data["delta_x_mm"], "Δx (mm)", "C0"),
        (axs[1], data["delta_y_mm"], "Δy (mm)", "C1"),
        (axs[2], np.rad2deg(data["delta_roll"]), "Δroll (°)", "C2"),
        (axs[3], np.rad2deg(data["delta_pitch"]), "Δpitch (°)", "C3"),
    ]
    for ax, y, lab, c in specs:
        ax.plot(t, y, color=c, lw=1.0)
        ax.set_ylabel(lab)
        ax.grid(True, alpha=0.3)
    axs[-1].set_xlabel("t (s)")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_overview(data: dict[str, Any], out_path: Path, title: str) -> None:
    plt = _setup_matplotlib()
    fig, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(data["t"], data["d_mm"], label="|Δ| mm")
    ax.plot(data["t"], data["e_mm"], label="EE err mm")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("t (s)")
    ax.set_title(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_ee_pose(
    data: dict[str, Any],
    i0: int,
    i1: int,
    out_path: Path,
    title: str,
) -> None:
    plt = _setup_matplotlib()
    tt = data["t"][i0:i1] - data["t"][i0]
    n = i1 - i0
    fig, axs = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    fig.suptitle(title)
    specs = [
        (axs[0, 0], "X (m)", data["target_x"][i0:i1], data["ee_x"][i0:i1]),
        (axs[0, 1], "Y (m)", data["target_y"][i0:i1], data["ee_y"][i0:i1]),
        (axs[1, 0], "Z (m)", data["target_z"][i0:i1], data["ee_z"][i0:i1]),
        (axs[1, 1], "Roll (rad)", np.full(n, data["des_roll"]), data["ee_roll"][i0:i1]),
        (axs[2, 0], "Pitch (rad)", np.full(n, data["des_pitch"]), data["ee_pitch"][i0:i1]),
        (axs[2, 1], "Yaw (rad)", np.full(n, data["des_yaw"]), data["ee_yaw"][i0:i1]),
    ]
    for ax, lab, tgt, act in specs:
        if not (np.all(np.isfinite(tgt)) and np.all(np.isfinite(act))):
            raise RuntimeError(f"non-finite samples in {lab} (bug: interp should fill)")
        ax.plot(tt, tgt, "k--", lw=1.2, label="Target")
        ax.plot(tt, act, "b-", lw=1.0, label="Actual")
        ax.set_ylabel(lab)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best")
    axs[2, 0].set_xlabel("t (s)")
    axs[2, 1].set_xlabel("t (s)")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_joints(
    data: dict[str, Any],
    i0: int,
    i1: int,
    out_path: Path,
    title: str,
) -> None:
    plt = _setup_matplotlib()
    tt = data["t"][i0:i1] - data["t"][i0]
    fig, axs = plt.subplots(4, 2, figsize=(11, 9), sharex=True)
    fig.suptitle(title)
    axs_flat = axs.ravel()
    for j in range(1, 8):
        ax = axs_flat[j - 1]
        ax.plot(tt, data["q_cmd"][j][i0:i1], "k--", label="q_cmd")
        ax.plot(tt, data["q_fb"][j][i0:i1], "b-", label="q_fb")
        ax.set_ylabel(f"j{j} (rad)")
        ax.grid(True, alpha=0.3)
        if j == 1:
            ax.legend(fontsize=8)
    axs_flat[-1].axis("off")
    axs_flat[-2].set_xlabel("t (s)")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_delta_vs_err(
    data: dict[str, Any],
    i0: int,
    i1: int,
    out_path: Path,
    title: str,
) -> None:
    plt = _setup_matplotlib()
    tt = data["t"][i0:i1] - data["t"][i0]
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(tt, data["d_mm"][i0:i1], label="|Δ| mm")
    ax.plot(tt, data["e_mm"][i0:i1], label="EE err mm")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("t (s)")
    ax.set_title(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_segment(
    data: dict[str, Any],
    i0: int,
    i1: int,
    out_dir: Path,
    tag: str,
    title_prefix: str,
    joints: bool = True,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = float(data["t"][i0])
    label = f"{title_prefix} · {tag} @t={t0:.1f}"
    plot_ee_pose(data, i0, i1, out_dir / "ee_pose_6panel.png", f"{label} · EE pose")
    plot_delta_vs_err(data, i0, i1, out_dir / "delta_vs_err.png", f"{label} · Δ vs err")
    if joints:
        plot_joints(data, i0, i1, out_dir / "joints_cmd_fb.png", f"{label} · joints")


def generate_plots(
    run_dir: Path,
    *,
    window_sec: float = 5.0,
    title_prefix: str = "",
    joints: bool = True,
    dt_force: float | None = None,
) -> dict[str, Any]:
    data = load_uniform_grid(run_dir, dt_force=dt_force)
    prefix = title_prefix or run_dir.name
    win = max(2, int(round(window_sec / data["dt"])))
    lon0, lon1 = _best_window(np.abs(data["dx_mm"]) - 0.3 * np.abs(data["dy_mm"]), win)
    lat0, lat1 = _best_window(np.abs(data["dy_mm"]) - 0.3 * np.abs(data["dx_mm"]), win)
    pk0, pk1 = _best_window(data["d_mm"], win)

    plot_overview(
        data,
        run_dir / "overview_delta_err.png",
        f"{prefix} · Δ vs EE err (continuous)",
    )
    plot_segment(data, lon0, lon1, run_dir / "plots_lon5s", "lon", prefix, joints)
    plot_segment(data, lat0, lat1, run_dir / "plots_lat5s", "lat", prefix, joints)
    plot_segment(data, pk0, pk1, run_dir / "plots_peak5s", "peakΔ", prefix, joints)

    summary = {
        "run": run_dir.name,
        "duration_s": float(data["t"][-1]),
        "dt_s": data["dt"],
        "e_rms_mm": float(np.sqrt(np.nanmean(data["e_mm"] ** 2))),
        "e_mean_mm": float(np.nanmean(data["e_mm"])),
        "e_max_mm": float(np.nanmax(data["e_mm"])),
        "d_mean_mm": float(np.nanmean(data["d_mm"])),
        "d_max_mm": float(np.nanmax(data["d_mm"])),
        "windows": {
            "lon_s": [float(data["t"][lon0]), float(data["t"][lon1 - 1])],
            "lat_s": [float(data["t"][lat0]), float(data["t"][lat1 - 1])],
            "peak_s": [float(data["t"][pk0]), float(data["t"][pk1 - 1])],
        },
        "outputs": [
            "overview_delta_err.png",
            "plots_lon5s/",
            "plots_lat5s/",
            "plots_peak5s/",
        ],
    }
    (run_dir / "plot_summary.json").write_text(
        __import__("json").dumps(summary, indent=2) + "\n"
    )
    return summary


def generate_axis_plots(
    run_dir: Path,
    *,
    max_window_sec: float = 5.0,
    min_window_sec: float = 1.0,
    title_prefix: str = "",
    joints: bool = True,
    dt_force: float | None = None,
) -> dict[str, Any]:
    """Four cases: X/Y translation and roll/pitch (about X/Y), each ≤ max_window_sec."""
    data = load_uniform_grid(run_dir, dt_force=dt_force)
    prefix = title_prefix or run_dir.name
    dt = float(data["dt"])

    dx = data["delta_x_mm"]
    dy = data["delta_y_mm"]
    dz = data["delta_z_mm"]
    dr = data["delta_roll"]
    dp = data["delta_pitch"]
    dyaw = data["delta_yaw"]

    cases = [
        (
            "tx",
            "X平移",
            _axis_dominance_score(dx, dy, dz),
            dx,
            "Δx",
            "mm",
        ),
        (
            "ty",
            "Y平移",
            _axis_dominance_score(dy, dx, dz),
            dy,
            "Δy",
            "mm",
        ),
        (
            "rx",
            "绕X旋转(roll)",
            _axis_dominance_score(dr, dp, dyaw),
            np.rad2deg(dr),
            "Δroll",
            "deg",
        ),
        (
            "ry",
            "绕Y旋转(pitch)",
            _axis_dominance_score(dp, dr, dyaw),
            np.rad2deg(dp),
            "Δpitch",
            "deg",
        ),
    ]

    plot_axis_overview(
        data,
        run_dir / "overview_axis_delta.png",
        f"{prefix} · Δx/Δy/Δroll/Δpitch",
    )
    plot_overview(
        data,
        run_dir / "overview_delta_err.png",
        f"{prefix} · Δ vs EE err (continuous)",
    )

    windows: dict[str, Any] = {}
    outputs = ["overview_axis_delta.png", "overview_delta_err.png"]
    for tag, name, score, primary, plab, punit in cases:
        i0, i1 = _best_active_window(
            score, dt=dt, max_sec=max_window_sec, min_sec=min_window_sec
        )
        out_dir = run_dir / f"plots_{tag}"
        t0 = float(data["t"][i0])
        t1 = float(data["t"][i1 - 1])
        dur = t1 - t0
        label = f"{prefix} · {name} @t={t0:.1f} ({dur:.1f}s)"
        plot_axis_disturbance(
            data,
            i0,
            i1,
            out_dir / "axis_vs_err.png",
            f"{label} · {plab} vs EE err",
            primary,
            plab,
            punit,
        )
        plot_ee_pose(data, i0, i1, out_dir / "ee_pose_6panel.png", f"{label} · EE pose")
        plot_delta_vs_err(
            data, i0, i1, out_dir / "delta_vs_err.png", f"{label} · |Δ| vs err"
        )
        if joints:
            plot_joints(data, i0, i1, out_dir / "joints_cmd_fb.png", f"{label} · joints")
        windows[tag] = {
            "name": name,
            "t_s": [t0, t1],
            "duration_s": dur,
            "primary_abs_mean": float(np.nanmean(np.abs(primary[i0:i1]))),
            "primary_abs_max": float(np.nanmax(np.abs(primary[i0:i1]))),
            "e_rms_mm": float(np.sqrt(np.nanmean(data["e_mm"][i0:i1] ** 2))),
        }
        outputs.append(f"plots_{tag}/")

    summary = {
        "run": run_dir.name,
        "duration_s": float(data["t"][-1]),
        "dt_s": dt,
        "e_rms_mm": float(np.sqrt(np.nanmean(data["e_mm"] ** 2))),
        "windows": windows,
        "outputs": outputs,
        "mode": "axis",
        "max_window_sec": max_window_sec,
        "min_window_sec": min_window_sec,
    }
    (run_dir / "plot_axis_summary.json").write_text(
        __import__("json").dumps(summary, indent=2) + "\n"
    )
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=str, default="", help="Run dir or name under data/runs/")
    ap.add_argument("--latest", action="store_true", help="Use newest run with aligned.csv")
    ap.add_argument("--window-sec", type=float, default=5.0, help="Segment length (s)")
    ap.add_argument("--title-prefix", type=str, default="", help="Plot title prefix")
    ap.add_argument("--no-joints", action="store_true", help="Skip joints_cmd_fb plots")
    ap.add_argument("--dt", type=float, default=0.0, help="Force resample dt (s); 0=auto")
    ap.add_argument(
        "--axes",
        action="store_true",
        help="Plot X/Y translation and roll/pitch windows (each ≤ --window-sec)",
    )
    ap.add_argument(
        "--min-window-sec",
        type=float,
        default=1.0,
        help="With --axes: minimum window length (s)",
    )
    args = ap.parse_args()

    run_dir = resolve_run(args.run, args.latest)
    if args.axes:
        summary = generate_axis_plots(
            run_dir,
            max_window_sec=args.window_sec,
            min_window_sec=args.min_window_sec,
            title_prefix=args.title_prefix,
            joints=not args.no_joints,
            dt_force=args.dt if args.dt > 0 else None,
        )
        print(f"Plotted axes {run_dir}")
        print(f"  duration={summary['duration_s']:.1f}s  e_rms={summary['e_rms_mm']:.1f} mm")
        for tag, w in summary["windows"].items():
            a, b = w["t_s"]
            print(
                f"  {tag} ({w['name']}): t={a:.1f}–{b:.1f}s ({w['duration_s']:.1f}s)  "
                f"|p|_max={w['primary_abs_max']:.2f}  e_rms={w['e_rms_mm']:.1f} mm"
            )
        print(f"  wrote overview_axis_delta + plots_tx/ty/rx/ry + plot_axis_summary.json")
        return 0

    summary = generate_plots(
        run_dir,
        window_sec=args.window_sec,
        title_prefix=args.title_prefix,
        joints=not args.no_joints,
        dt_force=args.dt if args.dt > 0 else None,
    )
    print(f"Plotted {run_dir}")
    print(
        f"  duration={summary['duration_s']:.1f}s  "
        f"e_rms={summary['e_rms_mm']:.1f} mm  d_mean={summary['d_mean_mm']:.1f} mm"
    )
    for w, (a, b) in summary["windows"].items():
        print(f"  {w}: t={a:.1f}–{b:.1f}s")
    print(f"  wrote overview + plots_lon/lat/peak + {run_dir / 'plot_summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())