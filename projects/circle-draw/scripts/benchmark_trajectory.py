#!/usr/bin/env python3
"""对比原版 move_arm_ik_demo 与 circle-draw 优化轨迹的跟踪品质。"""

from __future__ import annotations

import math
import sys
import time

import numpy as np

from _path_setup import _DEMO_DIR  # noqa: F401
from differential_ik import DifferentialIK, PrecomputedCircleTrajectory
from pinocchio_ik import PinocchioIK

CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
CIRCLE_RADIUS = 0.08
PERIOD_SEC = 8.0
ORIG_RATE_HZ = 50.0
OPT_RATE_HZ = 100.0
MAX_VEL = 0.2
CTRL_DT = 0.01


def circle_pos(t: float) -> np.ndarray:
    w = 2.0 * math.pi / PERIOD_SEC
    return CIRCLE_CENTER + CIRCLE_RADIUS * np.array([0.0, math.cos(w * t), math.sin(w * t)])


def circle_vel(t: float) -> np.ndarray:
    w = 2.0 * math.pi / PERIOD_SEC
    return CIRCLE_RADIUS * w * np.array([0.0, -math.sin(w * t), math.cos(w * t)])


def simulate_smooth_tracking(
    q_cmds: np.ndarray,
    dq_cmds: np.ndarray,
    cmd_dt: float,
    ik: PinocchioIK,
    max_vel: float = MAX_VEL,
) -> tuple[np.ndarray, np.ndarray]:
    """模拟 SmoothPositionController 限速 + 关节指令。"""
    n_cmd = len(q_cmds)
    steps_per_cmd = max(1, int(round(cmd_dt / CTRL_DT)))
    q_act = q_cmds[0].copy()
    pos_set = q_act.copy()
    ee_actual = []
    ee_target = []

    for i in range(n_cmd):
        q_cmd = q_cmds[i]
        for _ in range(steps_per_cmd):
            for j in range(q_act.shape[0]):
                err = q_cmd[j] - q_act[j]
                err = np.clip(err, -max_vel * CTRL_DT, max_vel * CTRL_DT)
                pos_set[j] += err
                pos_set[j] = np.clip(
                    pos_set[j],
                    min(q_cmd[j], q_act[j]),
                    max(q_cmd[j], q_act[j]),
                )
                q_act[j] = pos_set[j]
        p_act, _ = ik.forward(q_act)
        p_tgt, _ = ik.forward(q_cmd)
        ee_actual.append(p_act)
        ee_target.append(p_tgt)
    return np.array(ee_target), np.array(ee_actual)


def gen_original_commands(n_sec: float = PERIOD_SEC) -> tuple[np.ndarray, np.ndarray, float]:
    ik = PinocchioIK()
    dt = 1.0 / ORIG_RATE_HZ
    n = int(n_sec / dt)
    q_last, _ = ik.solve(circle_pos(0.0))
    qs = []
    dqs = []
    t0 = time.perf_counter()
    for i in range(n):
        t = i * dt
        q, _ = ik.solve(circle_pos(t), q_init=q_last)
        if i > 0:
            dqs.append((q - q_last) / dt)
        else:
            dqs.append(np.zeros_like(q))
        qs.append(q)
        q_last = q
    elapsed = time.perf_counter() - t0
    return np.array(qs), np.array(dqs), elapsed


def gen_diff_commands(n_sec: float = PERIOD_SEC, max_vel: float = 0.35) -> tuple[np.ndarray, np.ndarray, float]:
    tracker = DifferentialIK(pos_gain=10.0, max_dq=0.35)
    dt = 1.0 / OPT_RATE_HZ
    max_step = max_vel * dt
    n = int(n_sec / dt)
    q, _ = tracker.ik.solve(circle_pos(0.0))
    qs = []
    dqs = []
    t0 = time.perf_counter()
    for i in range(n):
        t = i * dt
        step = tracker.step(
            q, circle_pos(t), circle_vel(t), dt, max_joint_step=max_step)
        q = step.q
        qs.append(q)
        dqs.append(step.dq)
    elapsed = time.perf_counter() - t0
    return np.array(qs), np.array(dqs), elapsed


def gen_precompute_commands(
    n_sec: float = PERIOD_SEC, max_vel: float = 0.35,
) -> tuple[np.ndarray, np.ndarray, float]:
    dt = 1.0 / OPT_RATE_HZ
    max_step = max_vel * dt
    n = int(n_sec / dt)
    t0 = time.perf_counter()
    traj = PrecomputedCircleTrajectory(CIRCLE_CENTER, CIRCLE_RADIUS, PERIOD_SEC)
    build_time = time.perf_counter() - t0
    qs = []
    dqs = []
    q_prev = None
    for i in range(n):
        q, dq = traj.sample(i * dt, q_prev=q_prev, max_joint_step=max_step)
        q_prev = q.copy()
        qs.append(q)
        dqs.append(dq)
    return np.array(qs), np.array(dqs), build_time


def report(name: str, q_cmds: np.ndarray, dq_cmds: np.ndarray, gen_time: float, cmd_dt: float) -> None:
    ik = PinocchioIK()
    ee_tgt, ee_act = simulate_smooth_tracking(q_cmds, dq_cmds, cmd_dt, ik)
    err = np.linalg.norm(ee_tgt - ee_act, axis=1)
    req_speed = np.max(np.abs(np.diff(q_cmds, axis=0, prepend=q_cmds[:1]) / cmd_dt), axis=1)
    print(f'\n=== {name} ===')
    print(f'  command rate: {1.0/cmd_dt:.0f} Hz, frames: {len(q_cmds)}')
    print(f'  planner time: {gen_time*1000:.1f} ms')
    print(f'  required |dq| max: {req_speed.max():.3f} rad/s')
    print(f'  frames |dq|> {MAX_VEL}: {(req_speed > MAX_VEL).sum()}/{len(req_speed)}')
    print(f'  EE tracking error (sim): mean={err.mean()*1000:.2f} mm, max={err.max()*1000:.2f} mm')


def main() -> None:
    print('Benchmark: original move_arm_ik_demo vs circle-draw optimizations')
    print(f'Assumed SmoothPositionController max_velocity = {MAX_VEL} rad/s')
    q, dq, t = gen_original_commands()
    report('Original (50Hz iterative IK, v=0)', q, dq, t, 1.0 / ORIG_RATE_HZ)
    q, dq, t = gen_diff_commands(max_vel=MAX_VEL)
    report('Optimized diff IK (100Hz, rate-limited)', q, dq, t, 1.0 / OPT_RATE_HZ)
    q, dq, t = gen_precompute_commands(max_vel=MAX_VEL)
    report('Optimized precompute (100Hz, rate-limited)', q, dq, t, 1.0 / OPT_RATE_HZ)


if __name__ == '__main__':
    main()
