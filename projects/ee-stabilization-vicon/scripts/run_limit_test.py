#!/usr/bin/env python3
"""Automated amplitude/frequency limit tests for EE stabilization modes A/B/C."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from limit_test_lib import (  # noqa: E402
    CSV_FIELDS,
    DEFAULT_BASELINE_POS_RMS_MM,
    DisturbanceCase,
    MODES,
    RECORD_SEC,
    REPORT_ROOT,
    WARMUP_SEC,
    append_csv_row,
    build_phase_cases,
    compute_metrics,
    dedupe_cases,
    evaluate_failure,
    flatten_metrics_row,
    load_record_npz,
    load_metrics_row,
    plot_calibration,
    plot_grid_heatmap,
    plot_study_curves,
    save_metrics_json,
    save_record_npz,
    write_summary_report,
)

def _record_imports():
    from limit_test_record import measure_baseline_pos_rms, record_case
    return measure_baseline_pos_rms, record_case


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="EE stabilization disturbance limit test")
    parser.add_argument(
        "--phase",
        choices=["calibration", "amplitude", "frequency", "grid", "all", "quick", "baseline"],
        default="quick",
        help="Test phase (quick=smoke subset)",
    )
    parser.add_argument("--modes", nargs="+", default=["A", "B", "C"], choices=["A", "B", "C", "D"])
    parser.add_argument("--seeds", type=int, nargs="+", default=[42], help="Seeds (C++ 目前固定 42)")
    parser.add_argument("--report-dir", type=Path, default=REPORT_ROOT)
    parser.add_argument("--warmup", type=float, default=WARMUP_SEC)
    parser.add_argument("--record-sec", type=float, default=RECORD_SEC)
    parser.add_argument("--startup-wait", type=float, default=5.0)
    parser.add_argument(
        "--baseline-pos-rms-mm",
        type=float,
        default=None,
        help="Mode-A nominal pos RMS for T4; auto-measure if omitted with --measure-baseline",
    )
    parser.add_argument(
        "--measure-baseline",
        action="store_true",
        help="Run one nominal mode-A case to refresh baseline before sweep",
    )
    parser.add_argument("--skip-existing", action="store_true", help="Skip runs with existing npz")
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="Rebuild CSV/plots/report from existing npz under report-dir",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print planned cases only")
    return parser.parse_args()


def npz_path(report_dir: Path, case: DisturbanceCase) -> Path:
    return report_dir / "runs" / case.study / f"{case.run_id}.npz"


def apply_seeds(cases: list[DisturbanceCase], seeds: list[int]) -> list[DisturbanceCase]:
    out: list[DisturbanceCase] = []
    for case in cases:
        for seed in seeds:
            out.append(
                DisturbanceCase(
                    study=case.study,
                    mode=case.mode,
                    radius=case.radius,
                    orient_amp=case.orient_amp,
                    time_constant=case.time_constant,
                    amplitude_scale=case.amplitude_scale,
                    seed=seed,
                    scale=case.scale,
                )
            )
    return dedupe_cases(out)


def collect_rows_from_npz(report_dir: Path, baseline_pos_rms_mm: float) -> list[dict]:
    rows: list[dict] = []
    runs_dir = report_dir / "runs"
    if not runs_dir.exists():
        return rows
    for i, npz in enumerate(sorted(runs_dir.rglob("*.npz")), 1):
        json_path = npz.with_suffix(".json")
        if json_path.exists():
            rows.append(load_metrics_row(json_path))
            continue
        record = load_record_npz(npz)
        metrics = compute_metrics(record)
        failure = evaluate_failure(metrics, baseline_pos_rms_mm)
        save_metrics_json(metrics, failure, json_path)
        rows.append(flatten_metrics_row(metrics, failure))
        if i % 20 == 0:
            print(f"  metrics cache: {i} files", flush=True)
    return rows


def write_all_results_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    import csv

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def generate_plots(rows: list[dict], summary_dir: Path) -> None:
    plot_calibration(rows, summary_dir / "calibration_freq.png")
    plot_study_curves(rows, "S1_translation", "radius", "平移半径 (m)", summary_dir / "S1_translation_vs_radius.png")
    plot_study_curves(rows, "S2_rotation", "orient_amp", "姿态幅值 (rad)", summary_dir / "S2_rotation_vs_orient_amp.png")
    plot_study_curves(rows, "S3_scaled", "scale", "同比 scale", summary_dir / "S3_scaled_vs_scale.png")
    plot_study_curves(rows, "frequency", "time_constant", "T_c (s)", summary_dir / "frequency_vs_tc.png")
    for mode in ("A", "B", "C"):
        plot_grid_heatmap(rows, mode, summary_dir / f"grid_heatmap_{mode}.png")


def run_one_case(
    case: DisturbanceCase,
    report_dir: Path,
    warmup: float,
    record_sec: float,
    startup_wait: float,
    baseline_pos_rms_mm: float,
    csv_path: Path,
    skip_existing: bool,
) -> dict | None:
    out_npz = npz_path(report_dir, case)
    if skip_existing and out_npz.exists():
        json_path = out_npz.with_suffix(".json")
        if json_path.exists():
            row = load_metrics_row(json_path)
        else:
            record = load_record_npz(out_npz)
            metrics = compute_metrics(record)
            failure = evaluate_failure(metrics, baseline_pos_rms_mm)
            save_metrics_json(metrics, failure, json_path)
            row = flatten_metrics_row(metrics, failure)
        print(f"  skip existing {case.run_id} pos_rms={row['pos_rms_mm']:.3f}mm")
        return row

    print(
        f"  run {case.run_id} study={case.study} "
        f"R={case.radius:.3f} orient={case.orient_amp:.3f} Tc={case.time_constant:.3f}"
    )
    record_case_fn, = (_record_imports()[1],)
    record = record_case_fn(case, warmup_sec=warmup, record_sec=record_sec, startup_wait=startup_wait)
    save_record_npz(record, out_npz)
    metrics = compute_metrics(record)
    failure = evaluate_failure(metrics, baseline_pos_rms_mm)
    save_metrics_json(metrics, failure, out_npz.with_suffix(".json"))
    row = flatten_metrics_row(metrics, failure)
    append_csv_row(csv_path, row)
    print(
        f"    samples={row['samples']} pos_rms={row['pos_rms_mm']:.3f}mm "
        f"orient_rms={row['orient_rms_deg']:.3f}° fail={row['fail_level']} [{row['fail_codes']}]"
    )
    return row


def main() -> None:
    args = parse_args()
    report_dir = args.report_dir
    summary_dir = report_dir / "summary"
    csv_path = summary_dir / "all_results.csv"
    summary_dir.mkdir(parents=True, exist_ok=True)

    baseline = args.baseline_pos_rms_mm
    if args.measure_baseline and not args.analyze_only:
        measure_baseline_pos_rms, _ = _record_imports()
        print(">>> Measuring mode-A baseline @ nominal disturbance ...")
        baseline = measure_baseline_pos_rms()
        print(f"    baseline pos RMS = {baseline:.4f} mm")
    if baseline is None:
        baseline = DEFAULT_BASELINE_POS_RMS_MM

    baseline_path = summary_dir / "baseline.json"
    baseline_path.write_text(
        json.dumps({"baseline_pos_rms_mm": baseline}, indent=2),
        encoding="utf-8",
    )

    if args.analyze_only:
        rows = collect_rows_from_npz(report_dir, baseline)
        write_all_results_csv(rows, csv_path)
        generate_plots(rows, summary_dir)
        write_summary_report(rows, summary_dir / "limit_test_report.md", baseline)
        print(f"Analysis complete: {len(rows)} runs -> {summary_dir}")
        return

    if args.phase == "baseline":
        cases = apply_seeds(
            [
                DisturbanceCase(
                    study="baseline",
                    mode="A",
                    radius=0.35,
                    orient_amp=0.32,
                    time_constant=2.0,
                    scale=1.0,
                )
            ],
            args.seeds,
        )
    else:
        cases = apply_seeds(build_phase_cases(args.phase, args.modes), args.seeds)

    print(f"Phase={args.phase} modes={args.modes} cases={len(cases)} report_dir={report_dir}")
    if args.dry_run:
        for case in cases:
            print(f"  {case.study:16s} {case.mode} {case.run_id}")
        return

    rows: list[dict] = []
    for idx, case in enumerate(cases, 1):
        print(f"\n[{idx}/{len(cases)}] {MODES[case.mode]['label']}")
        try:
            row = run_one_case(
                case,
                report_dir,
                args.warmup,
                args.record_sec,
                args.startup_wait,
                baseline,
                csv_path,
                args.skip_existing,
            )
            if row is not None:
                rows.append(row)
        except Exception as exc:
            print(f"  ERROR: {exc}", file=sys.stderr)

    all_rows = collect_rows_from_npz(report_dir, baseline)
    write_all_results_csv(all_rows, csv_path)
    generate_plots(all_rows, summary_dir)
    write_summary_report(all_rows, summary_dir / "limit_test_report.md", baseline)
    print(f"\nDone. {len(all_rows)} runs summarized under {summary_dir}")


if __name__ == "__main__":
    main()
