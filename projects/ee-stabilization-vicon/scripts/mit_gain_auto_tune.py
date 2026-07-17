#!/usr/bin/env python3
"""自动调试 MIT j1–j3 的 kp/kd（真机）。

每轮：写 overlay → 重启 student_arm → 跑短轨迹(默认 cosine) → 打分 → 下一组。
增益只在节点启动时生效。

子命令:
  apply     只写 overlay YAML
  score     给已有 trajectory.csv / run 目录打分
  suggest   根据 trials.jsonl 建议下一组
  rank      打印排行榜
  auto      自动循环（需 --confirm-hw）

示例:
  python3 scripts/mit_gain_auto_tune.py apply --kp123 50 --kd123 1.5
  python3 scripts/mit_gain_auto_tune.py auto --confirm-hw --max-trials 8
  python3 scripts/mit_gain_auto_tune.py rank --session my_tune
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

from mit_gain_tune_lib import (
    DEFAULT_GRID,
    DEFAULT_TUNE_ROOT,
    MitProxGains,
    append_trial_log,
    best_from_log,
    expand_grid,
    format_score_table,
    load_trial_log,
    score_multiple_runs,
    score_trajectory_csv,
    suggest_next,
    write_student_overlay_yaml,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = PROJECT_ROOT.parents[1]
WS = ARM_ROOT / "windylab_ws"
MIT_TRAJ = PROJECT_ROOT / "data" / "mit_traj"


def _now() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _session_dir(name: str) -> Path:
    d = DEFAULT_TUNE_ROOT / name
    d.mkdir(parents=True, exist_ok=True)
    return d


def _kill_by_needle(needles: list[str]) -> None:
    self_pid = os.getpid()
    ppid = os.getppid()
    for needle in needles:
        try:
            out = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
        except subprocess.CalledProcessError:
            continue
        for line in out.splitlines():
            line = line.strip()
            if not line or needle not in line:
                continue
            if "mit_gain_auto_tune" in line:
                continue
            parts = line.split(None, 1)
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            if pid in (self_pid, ppid):
                continue
            print(f"[tune] kill pid={pid} ({needle})")
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
    time.sleep(1.0)


def _ros_env() -> dict[str, str]:
    env = os.environ.copy()
    # Ensure sourced env if caller already sourced; else prepend typical paths
    return env


def _resolve_cfg() -> tuple[Path, Path, Path, Path]:
    base = WS / "install" / "manipulator" / "share" / "manipulator" / "stabilization_hw_student_arm.yaml"
    if not base.is_file():
        base = WS / "src" / "arm-platform" / "config" / "stabilization_hw_student_arm.yaml"
    motor = WS / "install" / "manipulator" / "share" / "manipulator" / "motor_config.yaml"
    arm = WS / "install" / "manipulator" / "share" / "manipulator" / "arm_config.yaml"
    if not motor.is_file():
        motor = WS / "src" / "arm-platform" / "config" / "motor_config.yaml"
    if not arm.is_file():
        arm = WS / "src" / "arm-platform" / "config" / "arm_config.yaml"
    return base, motor, arm, base


def start_student(overlay: Path, port: str, log_path: Path) -> int:
    base, motor, arm, _ = _resolve_cfg()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ros2",
        "run",
        "manipulator",
        "student_arm_node",
        "--ros-args",
        "--params-file",
        str(base),
        "--params-file",
        str(overlay),
        "-p",
        "arm_type:=a_l1",
        "-p",
        "arm_version:=gamma",
        "-p",
        f"port_name:={port}",
        "-p",
        f"motor_config_path:={motor}",
        "-p",
        f"arm_config_path:={arm}",
    ]
    print("[tune] start student:", " ".join(cmd))
    with log_path.open("w") as logf:
        proc = subprocess.Popen(
            cmd,
            stdout=logf,
            stderr=subprocess.STDOUT,
            cwd=str(PROJECT_ROOT),
            env=_ros_env(),
            preexec_fn=os.setsid,
        )
    time.sleep(1.5)
    # Prefer C++ node pid
    try:
        out = subprocess.check_output(
            ["pgrep", "-n", "-f", "lib/manipulator/student_arm_node"], text=True
        ).strip()
        if out:
            return int(out.splitlines()[-1])
    except subprocess.CalledProcessError:
        pass
    return proc.pid


def stop_student(pid: Optional[int]) -> None:
    _kill_by_needle(
        [
            "lib/manipulator/student_arm_node",
            "hw_mit_traj_test.py",
        ]
    )
    if pid:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(1.0)


def wait_joint_states(timeout_s: float = 60.0) -> bool:
    t0 = time.time()
    n = 0
    while time.time() - t0 < timeout_s:
        n += 1
        r = subprocess.run(
            ["timeout", "5", "ros2", "topic", "echo", "/joint_states", "--once"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if r.returncode == 0:
            print(f"[tune] /joint_states OK (probe {n})")
            return True
        print(f"[tune] waiting joint_states ({n})...")
        time.sleep(0.5)
    return False


def claim_serial(port: str) -> None:
    script = f"""
set +u
source /opt/ros/humble/setup.bash
source "{WS}/install/setup.bash"
set -u
source "{ARM_ROOT}/scripts/resolve_arm_port.sh"
source "{ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "{port}"
"""
    subprocess.run(["bash", "-lc", script], check=False)


def run_traj_trial(
    task: str, trial_tag: str, return_zero: bool = True
) -> tuple[int, list[Path]]:
    """Run hw_mit_traj_test; return (rc, list of new run dirs)."""
    before = time.time()
    cmd = [
        sys.executable,
        str(PROJECT_ROOT / "scripts" / "hw_mit_traj_test.py"),
        "--task",
        task,
        "--confirm-hw",
    ]
    if return_zero:
        cmd.append("--return-zero")
    else:
        cmd.append("--no-return-zero")
    print("[tune] traj:", " ".join(cmd))
    rc = subprocess.run(cmd, cwd=str(PROJECT_ROOT), env=_ros_env()).returncode

    tasks = ["cosine", "line", "circle"] if task == "all" else [task]
    found: list[Path] = []
    if MIT_TRAJ.is_dir():
        for tname in tasks:
            cands = []
            for p in MIT_TRAJ.iterdir():
                if not p.is_dir():
                    continue
                if not p.name.endswith(f"_{tname}"):
                    continue
                try:
                    if p.stat().st_mtime >= before - 2.0 and (p / "trajectory.csv").is_file():
                        cands.append(p)
                except OSError:
                    continue
            if cands:
                run_dir = max(cands, key=lambda p: p.stat().st_mtime)
                (run_dir / "tune_trial_tag.txt").write_text(trial_tag + "\n")
                found.append(run_dir)
    return rc, found


def cmd_apply(args: argparse.Namespace) -> int:
    gains = MitProxGains(
        kp123=args.kp123,
        kd123=args.kd123,
        kp1=args.kp1,
        kp2=args.kp2,
        kp3=args.kp3,
        kd1=args.kd1,
        kd2=args.kd2,
        kd3=args.kd3,
    ).clamp()
    sess = _session_dir(args.session)
    path = sess / f"overlay_{gains.key()}.yaml"
    write_student_overlay_yaml(path, gains)
    print(f"wrote {path}")
    print(json.dumps(gains.to_dict(), indent=2))
    return 0


def cmd_score(args: argparse.Namespace) -> int:
    path = Path(args.run_dir).expanduser()
    csv_path = path / "trajectory.csv" if path.is_dir() else path
    sc = score_trajectory_csv(csv_path)
    print(json.dumps(sc.to_dict(), indent=2, ensure_ascii=False))
    return 0 if sc.valid else 1


def cmd_suggest(args: argparse.Namespace) -> int:
    sess = _session_dir(args.session)
    log = sess / "trials.jsonl"
    g = suggest_next(log, strategy=args.strategy)
    print(json.dumps(g.to_dict(), indent=2))
    print("key:", g.key())
    return 0


def cmd_rank(args: argparse.Namespace) -> int:
    sess = _session_dir(args.session)
    log = sess / "trials.jsonl"
    rows = load_trial_log(log)
    print(format_score_table(rows, top=args.top))
    best = best_from_log(log)
    if best:
        print("\nBEST:", best.get("key"), "score=", best.get("score"))
        print(json.dumps(best.get("gains", {}), indent=2))
        ov = sess / "overlay_best.yaml"
        write_student_overlay_yaml(ov, MitProxGains.from_dict(best.get("gains", {})))
        print("wrote", ov)
    return 0


def cmd_auto(args: argparse.Namespace) -> int:
    if not args.confirm_hw:
        print("Need --confirm-hw to move the arm.", file=sys.stderr)
        return 2

    sess = _session_dir(args.session)
    log = sess / "trials.jsonl"
    port = args.port or os.environ.get("PORT_NAME") or os.environ.get("ARM_SERIAL_PORT") or "/dev/ttyTHS3"

    print(f"[tune] session={sess}")
    print(
        f"[tune] port={port} task={args.task} strategy={args.strategy} "
        f"max_trials={args.max_trials} return_zero={args.return_zero}"
    )

    # Initial claim (best-effort; may need sudo for robot.service)
    claim_serial(port)

    student_pid: Optional[int] = None
    expected_n = 3 if args.task == "all" else 1
    try:
        for trial_i in range(1, args.max_trials + 1):
            gains = suggest_next(log, strategy=args.strategy)
            # skip if already tried (suggest should avoid, but be safe)
            if any(r.get("key") == gains.key() for r in load_trial_log(log)):
                # force grid leftover
                remaining = [
                    g
                    for g in expand_grid()
                    if g.key() not in {r.get("key") for r in load_trial_log(log)}
                ]
                if not remaining:
                    print("[tune] search space exhausted")
                    break
                gains = remaining[0]

            overlay = sess / f"overlay_{gains.key()}.yaml"
            write_student_overlay_yaml(overlay, gains)
            print(f"\n===== trial {trial_i}/{args.max_trials}  {gains.key()} =====")
            print(json.dumps(gains.to_dict(), indent=2))

            stop_student(student_pid)
            student_pid = None
            # re-claim if something grabbed serial
            claim_serial(port)

            slog = sess / f"student_{_now()}_{gains.key()}.log"
            student_pid = start_student(overlay, port, slog)
            if not wait_joint_states(90.0):
                print("[FAIL] no joint_states; see", slog)
                append_trial_log(
                    log,
                    {
                        "time": _now(),
                        "key": gains.key(),
                        "gains": gains.to_dict(),
                        "valid": False,
                        "score": 1e6,
                        "reason": "no_joint_states",
                        "student_log": str(slog),
                    },
                )
                continue

            tag = f"{_now()}_{gains.key()}"
            rc, run_dirs = run_traj_trial(
                args.task, tag, return_zero=args.return_zero
            )
            if len(run_dirs) < expected_n:
                append_trial_log(
                    log,
                    {
                        "time": _now(),
                        "key": gains.key(),
                        "gains": gains.to_dict(),
                        "valid": False,
                        "score": 1e6,
                        "reason": f"incomplete_runs n={len(run_dirs)} rc={rc}",
                        "traj_rc": rc,
                        "run_dirs": [str(p) for p in run_dirs],
                    },
                )
                print(f"[FAIL] expected {expected_n} runs, got {len(run_dirs)}")
                continue

            sc = (
                score_multiple_runs(run_dirs)
                if len(run_dirs) > 1
                else score_trajectory_csv(run_dirs[0] / "trajectory.csv")
            )
            trial = {
                "time": _now(),
                "key": gains.key(),
                "gains": gains.to_dict(),
                "run_dirs": [str(p) for p in run_dirs],
                "traj_rc": rc,
                **sc.to_dict(),
            }
            append_trial_log(log, trial)
            print(
                f"[tune] score={sc.score:.2f} valid={sc.valid} "
                f"ee_rms={sc.ee_rms_mm:.2f} ee_max={sc.ee_max_mm:.2f} "
                f"j123={sc.j123_mean_rad:.4f} chatter={sc.chatter:.2f}"
            )
            if sc.extras.get("task_scores"):
                print("[tune] per-task scores:", sc.extras["task_scores"])
            print(format_score_table(load_trial_log(log), top=8))

            if args.sleep_between > 0:
                time.sleep(args.sleep_between)
    finally:
        stop_student(student_pid)

    best = best_from_log(log)
    if best:
        ov = sess / "overlay_best.yaml"
        write_student_overlay_yaml(ov, MitProxGains.from_dict(best["gains"]))
        summary = {
            "best_key": best.get("key"),
            "best_score": best.get("score"),
            "best_gains": best.get("gains"),
            "ee_rms_mm": best.get("ee_rms_mm"),
            "ee_max_mm": best.get("ee_max_mm"),
            "j123_mean_rad": best.get("j123_mean_rad"),
            "run_dirs": best.get("run_dirs"),
            "overlay": str(ov),
        }
        (sess / "best_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
        )
        print("\n===== BEST =====")
        print(best.get("key"), "score=", best.get("score"))
        print("p_gain:", best.get("gains", {}).get("p_gain"))
        print("d_gain:", best.get("gains", {}).get("d_gain"))
        print("overlay:", ov)
        print(format_score_table(load_trial_log(log), top=15))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_session(sp: argparse.ArgumentParser) -> None:
        sp.add_argument(
            "--session",
            type=str,
            default="prox_j123_all3",
            help="Tune session name under data/tune_mit_gains/",
        )

    ap = sub.add_parser("apply", help="Write overlay YAML only")
    add_session(ap)
    ap.add_argument("--kp123", type=float, default=30.0)
    ap.add_argument("--kd123", type=float, default=1.0)
    ap.add_argument("--kp1", type=float, default=None)
    ap.add_argument("--kp2", type=float, default=None)
    ap.add_argument("--kp3", type=float, default=None)
    ap.add_argument("--kd1", type=float, default=None)
    ap.add_argument("--kd2", type=float, default=None)
    ap.add_argument("--kd3", type=float, default=None)
    ap.set_defaults(func=cmd_apply)

    sc = sub.add_parser("score", help="Score a run dir or trajectory.csv")
    add_session(sc)
    sc.add_argument("--run-dir", required=True)
    sc.set_defaults(func=cmd_score)

    sg = sub.add_parser("suggest", help="Suggest next gains")
    add_session(sg)
    sg.add_argument("--strategy", choices=("coord", "grid"), default="coord")
    sg.set_defaults(func=cmd_suggest)

    rk = sub.add_parser("rank", help="Show leaderboard")
    add_session(rk)
    rk.add_argument("--top", type=int, default=15)
    rk.set_defaults(func=cmd_rank)

    au = sub.add_parser("auto", help="Auto restart + traj + score loop")
    add_session(au)
    au.add_argument("--confirm-hw", action="store_true")
    au.add_argument("--max-trials", type=int, default=16)
    au.add_argument(
        "--task",
        choices=("cosine", "line", "circle", "all"),
        default="all",
        help="Default all = cosine+line+circle, return-to-zero between each",
    )
    au.add_argument("--strategy", choices=("coord", "grid"), default="grid")
    au.add_argument("--port", type=str, default="")
    au.add_argument("--sleep-between", type=float, default=1.0)
    au.add_argument(
        "--return-zero",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Return to q=0 after each motion (default: on)",
    )
    au.set_defaults(func=cmd_auto)

    return p


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
