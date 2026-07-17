#!/usr/bin/env python3
"""Mode C 自动调参（人机协作）。

真机增益只在节点启动时加载，因此每轮调参 = 写 overlay → 重启 Mode C → 录制 → 打分。

子命令:
  score     给已有 data/runs 打分、排名
  suggest   根据历史 trials 建议下一组参数
  apply     只生成 overlay YAML（可配合 PARAMS_OVERLAY=... ./run_hw.sh C）
  auto      交互自动循环（推荐）：启动/录制/提示晃机/停录/打分/下一组

示例见: ./run_tune_mode_c.sh --help
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from mode_c_tune_lib import (
    DEFAULT_GRID,
    DEFAULT_TUNE_ROOT,
    ModeCParams,
    append_trial_log,
    best_from_log,
    expand_grid,
    format_score_table,
    load_trial_log,
    score_aligned_csv,
    suggest_next,
    write_overlay_yaml,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
RUN_HW = PROJECT_ROOT / "run_hw.sh"
RUN_RECORD = PROJECT_ROOT / "run_record.sh"
DATA_RUNS = PROJECT_ROOT / "data" / "runs"


def _now_tag() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _pkill_stack() -> None:
    patterns = [
        "record_failure_analysis",
        "ros2 bag record",
        "ee_stabilization",
        "student_arm_node",
        "vicon_relative",
        "stabilization_vicon",
        "vrpn_listener",
        "move_to_home",
        "run_hw.sh",
    ]
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pat in patterns:
            subprocess.run(
                ["pkill", f"-{sig.name.replace('SIG', '')}", "-f", pat],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        time.sleep(0.4 if sig != signal.SIGKILL else 0.8)


def _wait_ready(log_path: Path, timeout_s: float = 90.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if log_path.is_file():
            text = log_path.read_text(errors="ignore")
            if (
                "process has died" in text
                or "AL1::Init is not" in text
                or "InvalidParameterTypeException" in text
                or "has invalid type" in text
            ):
                # extract last useful error line
                for line in reversed(text.splitlines()):
                    if any(
                        k in line
                        for k in (
                            "invalid type",
                            "process has died",
                            "InvalidParameter",
                            "what():",
                            "ERROR",
                        )
                    ):
                        print("[FAIL]", line.strip()[:200])
                        break
                print("[FAIL] launch error — see", log_path)
                return False
            if "Latched t0 plane pose" in text and "hw_torque_lpf_alpha=" in text:
                return True
        time.sleep(1.0)
    print("[FAIL] timeout waiting for Mode C ready:", log_path)
    return False


def _latest_run_dir(after_ts: float) -> Optional[Path]:
    if not DATA_RUNS.is_dir():
        return None
    cands = []
    for p in DATA_RUNS.iterdir():
        if not p.is_dir():
            continue
        try:
            mtime = p.stat().st_mtime
        except OSError:
            continue
        if mtime >= after_ts - 2.0:
            cands.append((mtime, p))
    if not cands:
        return None
    cands.sort()
    return cands[-1][1]


def _prompt(msg: str) -> str:
    try:
        return input(msg)
    except EOFError:
        return ""


def cmd_score(args: argparse.Namespace) -> int:
    runs = args.runs or []
    if args.all_recent:
        if DATA_RUNS.is_dir():
            dirs = sorted(
                [p for p in DATA_RUNS.iterdir() if p.is_dir()],
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            runs = [str(p) for p in dirs[: args.all_recent]]
    if not runs:
        print("请指定 --run DIR，或 --all-recent N", file=sys.stderr)
        return 2

    scored = []
    for r in runs:
        path = Path(r)
        if not path.is_absolute():
            path = DATA_RUNS / path if not path.exists() else path
        s = score_aligned_csv(path)
        # params unknown offline — use placeholder from note/key if any
        p = ModeCParams()
        meta = path / "run_meta.json"
        if meta.is_file():
            note = json.loads(meta.read_text()).get("note", "")
            print(f"\n[{path.name}] note={note}")
        print(
            f"  valid={s.valid} score={s.score:.2f} reason={s.reason}\n"
            f"  e_strong={s.e_strong_med_mm:.1f}mm  e_static={s.e_static_med_mm:.1f}mm  "
            f"e/Δ={s.ratio_strong_med:.2f}  v12={s.v12_rms:.2f}  "
            f"Δmax={s.delta_max_mm:.0f} emax={s.e_max_mm:.0f}"
        )
        scored.append((p, s))

    print("\n" + format_score_table(scored))
    return 0


def cmd_suggest(args: argparse.Namespace) -> int:
    session = Path(args.session)
    trials = load_trial_log(session)
    tried = {ModeCParams.from_dict(t["params"]).key() for t in trials if "params" in t}
    best = best_from_log(trials) or ModeCParams.from_dict(
        {
            "kp_pos": args.kp_pos,
            "kp_ori": args.kp_ori,
            "kd_pos": args.kd_pos,
            "kd_ori": args.kd_ori,
            "osc_lambda": args.osc_lambda,
            "hw_torque_lpf_alpha": args.lpf_alpha,
        }
    )
    nxt = suggest_next(
        strategy=args.strategy,
        base=best,
        tried=tried,
        trial_index=len(trials),
        seed=args.seed,
    )
    print("current best:", best.key() if trials else "(default)")
    print("next:", nxt.key())
    print(json.dumps(nxt.clamp().__dict__, indent=2))
    if args.write:
        out = session / f"suggest_{nxt.key()}.yaml"
        write_overlay_yaml(out, nxt)
        print("wrote", out)
    return 0


def cmd_apply(args: argparse.Namespace) -> int:
    p = ModeCParams(
        kp_pos=args.kp_pos,
        kp_ori=args.kp_ori,
        kd_pos=args.kd_pos,
        kd_ori=args.kd_ori,
        osc_lambda=args.osc_lambda,
        hw_torque_lpf_alpha=args.lpf_alpha,
        hw_torque_limit=args.torque_limit,
    ).clamp()
    out = Path(args.out)
    write_overlay_yaml(out, p)
    print(f"overlay: {out.resolve()}")
    print(f"params:  {p.key()}")
    print(
        f"\n启动:\n  PARAMS_OVERLAY={out.resolve()} ./run_hw.sh C\n"
        f"录制:\n  ./run_record.sh --duration 0 --mode C --note {p.key()}"
    )
    return 0


def cmd_list_grid(args: argparse.Namespace) -> int:
    base = ModeCParams(
        kp_pos=args.kp_pos,
        kp_ori=args.kp_ori,
        kd_pos=args.kd_pos,
        kd_ori=args.kd_ori,
        osc_lambda=args.osc_lambda,
        hw_torque_lpf_alpha=args.lpf_alpha,
    )
    pts = expand_grid(base=base, max_points=args.max_points)
    for i, p in enumerate(pts, 1):
        print(f"{i:3d} {p.key()}")
    print(f"total={len(pts)} (cap={args.max_points})")
    return 0


def _start_hw(overlay: Path, log_path: Path) -> subprocess.Popen:
    env = os.environ.copy()
    env["PARAMS_OVERLAY"] = str(overlay.resolve())
    env["START_VRPN"] = env.get("START_VRPN", "true")
    env["HOME_BEFORE_STABILIZE"] = env.get("HOME_BEFORE_STABILIZE", "true")
    env["ARM_MAX_VELOCITY"] = env.get("ARM_MAX_VELOCITY", "0.2")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        ["bash", str(RUN_HW), "C"],
        cwd=str(PROJECT_ROOT),
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _start_record(note: str, log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        [
            "bash",
            str(RUN_RECORD),
            "--duration",
            "0",
            "--mode",
            "C",
            "--note",
            note,
            "--no-stdin-events",
            "--autosave-sec",
            "20",
        ],
        cwd=str(PROJECT_ROOT),
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _stop_proc_group(proc: Optional[subprocess.Popen], sig: int = signal.SIGINT) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def cmd_auto(args: argparse.Namespace) -> int:
    session = Path(args.session)
    if not session.is_absolute():
        session = DEFAULT_TUNE_ROOT / session
    session.mkdir(parents=True, exist_ok=True)
    (session / "overlays").mkdir(exist_ok=True)

    trials = load_trial_log(session)
    tried = {
        ModeCParams.from_dict(t["params"]).key()
        for t in trials
        if "params" in t and not t.get("failed")
    }
    best = best_from_log(trials) or ModeCParams(
        kp_pos=args.kp_pos,
        kp_ori=args.kp_ori,
        kd_pos=args.kd_pos,
        kd_ori=args.kd_ori,
        osc_lambda=args.osc_lambda,
        hw_torque_lpf_alpha=args.lpf_alpha,
    ).clamp()

    print(
        f"\n=== Mode C auto-tune session: {session} ===\n"
        f"strategy={args.strategy}  max_trials={args.max_trials}\n"
        f"start_best={best.key()}  already_tried={len(tried)}\n"
        f"每轮请: 晃机 ~15–25s → 放回原位静止 ≥15s → 按 Enter\n"
    )
    if args.dry_run:
        for i in range(min(5, args.max_trials)):
            nxt = suggest_next(
                args.strategy, best, tried, trial_index=len(tried) + i, seed=args.seed
            )
            print(f"  would try: {nxt.key()}")
            tried.add(nxt.key())
        return 0

    for trial_i in range(args.max_trials):
        nxt = suggest_next(
            strategy=args.strategy,
            base=best,
            tried=tried,
            trial_index=len(trials) + trial_i,
            seed=args.seed,
        )
        if nxt.key() in tried and args.strategy == "grid":
            print("grid exhausted.")
            break

        overlay = session / "overlays" / f"trial_{len(trials)+trial_i:03d}_{nxt.key()}.yaml"
        write_overlay_yaml(overlay, nxt)
        print(f"\n----- trial {trial_i+1}/{args.max_trials}: {nxt.key()} -----")
        print(f"overlay: {overlay}")

        ans = _prompt("Enter=开始本轮, s=跳过, q=结束 > ").strip().lower()
        if ans == "q":
            break
        if ans == "s":
            tried.add(nxt.key())
            append_trial_log(
                session,
                {
                    "ts": _now_tag(),
                    "params": nxt.__dict__,
                    "skipped": True,
                },
            )
            continue

        _pkill_stack()
        time.sleep(1.0)
        hw_log = session / f"hw_{_now_tag()}.log"
        rec_log = session / f"rec_{_now_tag()}.log"
        t_start = time.time()
        hw_proc = _start_hw(overlay, hw_log)
        if not _wait_ready(hw_log, timeout_s=args.ready_timeout):
            _stop_proc_group(hw_proc, signal.SIGTERM)
            _pkill_stack()
            append_trial_log(
                session,
                {
                    "ts": _now_tag(),
                    "params": nxt.__dict__,
                    "failed": True,
                    "reason": "launch_error",
                    "hw_log": str(hw_log),
                },
            )
            # Do not mark as tried — allow retry after fixing infra/YAML issues.
            print("[INFO] 本轮启动失败，参数可重试（未记入已完成）。")
            continue

        print("Mode C ready. 开始录制…")
        rec_proc = _start_record(nxt.key(), rec_log)
        time.sleep(2.0)
        print(
            "\n>>> 请晃飞机 15–25 s，然后放回原位静止 ≥15 s。\n"
            ">>> 完成后按 Enter 结束本轮录制。\n"
        )
        if args.auto_duration > 0:
            print(f"(或等待 --auto-duration={args.auto_duration}s)")
            # wait for Enter with timeout
            import select

            r, _, _ = select.select([sys.stdin], [], [], float(args.auto_duration))
            if not r:
                print("auto-duration reached.")
            else:
                sys.stdin.readline()
        else:
            _prompt("")

        # stop recorder first (INT for clean save)
        _stop_proc_group(rec_proc, signal.SIGINT)
        try:
            rec_proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            _stop_proc_group(rec_proc, signal.SIGKILL)
        time.sleep(1.0)
        _stop_proc_group(hw_proc, signal.SIGTERM)
        time.sleep(1.0)
        _pkill_stack()

        run_dir = _latest_run_dir(t_start)
        if run_dir is None:
            print("[WARN] 找不到新的 data/runs 目录")
            sc = None
        else:
            # wait for aligned.csv
            for _ in range(20):
                if (run_dir / "aligned.csv").is_file():
                    break
                time.sleep(0.5)
            sc = score_aligned_csv(run_dir)
            print(
                f"run: {run_dir.name}\n"
                f"score={sc.score:.2f} valid={sc.valid} ({sc.reason})\n"
                f"  e_strong={sc.e_strong_med_mm:.1f}mm  e_static={sc.e_static_med_mm:.1f}mm  "
                f"e/Δ={sc.ratio_strong_med:.2f}  v12={sc.v12_rms:.2f}"
            )

        record = {
            "ts": _now_tag(),
            "params": nxt.__dict__,
            "overlay": str(overlay),
            "run_dir": str(run_dir) if run_dir else None,
            "score": sc.to_dict() if sc else None,
            "hw_log": str(hw_log),
            "rec_log": str(rec_log),
        }
        append_trial_log(session, record)
        tried.add(nxt.key())
        trials = load_trial_log(session)
        new_best = best_from_log(trials)
        if new_best is not None:
            best = new_best
            print(f"best so far: {best.key()}")

        # write best overlay
        write_overlay_yaml(session / "best_overlay.yaml", best)
        with (session / "best_params.json").open("w", encoding="utf-8") as f:
            json.dump(best.__dict__, f, indent=2)

    print(f"\nDone. session={session}")
    print(f"trials log: {session / 'trials.jsonl'}")
    print(f"best overlay: {session / 'best_overlay.yaml'}")
    print(
        "采用最佳参数:\n"
        f"  PARAMS_OVERLAY={session / 'best_overlay.yaml'} ./run_hw.sh C\n"
        "或把 overlay 内容合并进 stabilization_hw_mode_c.yaml"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Mode C auto-tune (human-in-the-loop)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_param_args(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--kp-pos", type=float, default=600.0)
        sp.add_argument("--kp-ori", type=float, default=450.0)
        sp.add_argument("--kd-pos", type=float, default=100.0)
        sp.add_argument("--kd-ori", type=float, default=130.0)
        sp.add_argument("--osc-lambda", type=float, default=0.05)
        sp.add_argument("--lpf-alpha", type=float, default=0.55)
        sp.add_argument("--torque-limit", type=float, default=6.0)

    sp = sub.add_parser("score", help="Score existing run directories")
    sp.add_argument("--run", dest="runs", action="append", default=[])
    sp.add_argument("--all-recent", type=int, default=0, help="Score N newest runs")
    sp.set_defaults(func=cmd_score)

    sp = sub.add_parser("suggest", help="Suggest next params from session log")
    sp.add_argument("--session", type=str, required=True)
    sp.add_argument(
        "--strategy",
        choices=("grid", "coordinate", "random"),
        default="coordinate",
    )
    sp.add_argument("--seed", type=int, default=0)
    sp.add_argument("--write", action="store_true")
    add_param_args(sp)
    sp.set_defaults(func=cmd_suggest)

    sp = sub.add_parser("apply", help="Write overlay YAML only")
    sp.add_argument(
        "--out",
        type=str,
        default=str(DEFAULT_TUNE_ROOT / "manual_overlay.yaml"),
    )
    add_param_args(sp)
    sp.set_defaults(func=cmd_apply)

    sp = sub.add_parser("grid", help="List grid candidates")
    sp.add_argument("--max-points", type=int, default=64)
    add_param_args(sp)
    sp.set_defaults(func=cmd_list_grid)

    sp = sub.add_parser("auto", help="Interactive auto-tune loop (recommended)")
    sp.add_argument(
        "--session",
        type=str,
        default=f"mode_c_{_now_tag()}",
        help="Session name under data/tune/",
    )
    sp.add_argument(
        "--strategy",
        choices=("grid", "coordinate", "random"),
        default="coordinate",
    )
    sp.add_argument("--max-trials", type=int, default=8)
    sp.add_argument("--seed", type=int, default=0)
    sp.add_argument("--ready-timeout", type=float, default=120.0)
    sp.add_argument(
        "--auto-duration",
        type=float,
        default=0.0,
        help="If >0, end trial after N seconds without waiting Enter",
    )
    sp.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print next candidates, do not launch HW",
    )
    add_param_args(sp)
    sp.set_defaults(func=cmd_auto)

    return p


def main() -> int:
    # allow `python3 scripts/mode_c_auto_tune.py` without PYTHONPATH
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
