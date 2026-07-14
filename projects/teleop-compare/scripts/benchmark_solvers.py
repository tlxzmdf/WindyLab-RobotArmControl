#!/usr/bin/env python3
"""微基准：CLIK 右伪逆 vs WBC 同构求解器（单步 / 10 子步）。"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

ARM_ROOT = Path(__file__).resolve().parents[3]
URDF = (
    ARM_ROOT / 'windylab_ws' / 'src' / 'arm_ee_stabilization_description'
    / 'urdf' / 'single_arm.urdf'
)
SUBSTEPS = 10
N = 5000


def build_j(q: np.ndarray, model: pin.Model, data: pin.Data, ee_id: int) -> np.ndarray:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    J_lwa = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)
    J_loc = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL)
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = J_lwa[:3, i]
        J[3:, i] = J_loc[3:, i]
    return J


def solve_clik(J, twist_cmd, damping, null_gain, q_err, max_vel):
    lhs = J @ J.T + (damping ** 2) * np.eye(6)
    jsharp = J.T @ np.linalg.solve(lhs, np.eye(6))
    dq = jsharp @ twist_cmd + null_gain * (np.eye(6) - jsharp @ J) @ q_err
    return np.clip(dq, -max_vel, max_vel)


def solve_wbc_same(J, twist_cmd, damping, null_gain, q_err, max_vel):
    return solve_clik(J, twist_cmd, damping, null_gain, q_err, max_vel)


def bench_one(fn, J, twist, damp, ns, qerr, mv):
    t0 = time.perf_counter()
    for _ in range(N):
        fn(J, twist, damp, ns, qerr, mv)
    return (time.perf_counter() - t0) / N * 1e6


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    ee_id = model.getFrameId('link6')
    rng = np.random.default_rng(42)

    q = pin.neutral(model)
    q[1], q[2], q[4] = 0.35, -0.55, 0.45
    J = build_j(q, model, data, ee_id)
    twist = np.array([32, 32, 32, 24, 24, 24]) * 0.01
    qerr = rng.normal(0, 0.05, 6)
    damp, ns, mv = 0.035, 0.25, 4.5

    us_clik = bench_one(solve_clik, J, twist, damp, ns, qerr, mv)
    us_wbc = bench_one(solve_wbc_same, J, twist, damp, ns, qerr, mv)

    result = {
        'iterations_per_call': N,
        'substeps_per_control_cycle': SUBSTEPS,
        'single_solve_us': {
            'clik': us_clik,
            'wbc_same_kernel': us_wbc,
        },
        'full_cycle_estimate_us': {
            'clik_10_substeps': us_clik * SUBSTEPS,
            'wbc_10_substeps': us_wbc * SUBSTEPS,
        },
        'note': 'WBC 额外开销来自每子步 FK+Jacobian+积分状态；内核与 CLIK 相同',
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
