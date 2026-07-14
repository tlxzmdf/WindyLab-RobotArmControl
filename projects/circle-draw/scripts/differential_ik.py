#!/usr/bin/env python3
"""笛卡尔速度 IK：单步阻尼伪逆，适合实时轨迹跟踪。

与 demo/pinocchio_ik.py 的迭代位置 IK 不同，本模块每控制周期只做
一次 Jacobian 步进，并输出关节速度前馈，避免 50Hz 位置阶跃 + 限速
控制器造成的跟踪滞后。
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from _path_setup import _DEMO_DIR  # noqa: F401 — side effect: sys.path
from pinocchio_ik import DEFAULT_URDF, EE_FRAME, PinocchioIK


@dataclass
class DiffIkStep:
    q: np.ndarray
    dq: np.ndarray
    position_error: float
    success: bool


class DifferentialIK:
    """笛卡尔速度 IK + 位置误差反馈。"""

    def __init__(
        self,
        urdf_path: str = DEFAULT_URDF,
        ee_frame: str = EE_FRAME,
        damp: float = 1e-4,
        pos_gain: float = 8.0,
        max_dq: float = 0.35,
    ):
        self.ik = PinocchioIK(urdf_path=urdf_path, ee_frame=ee_frame)
        self.model = self.ik.model
        self.data = self.ik.data
        self.frame_id = self.ik.frame_id
        self.q_lower = self.ik.q_lower
        self.q_upper = self.ik.q_upper
        self.damp = damp
        self.pos_gain = pos_gain
        self.max_dq = max_dq

    def forward_position(self, q: np.ndarray) -> np.ndarray:
        pos, _ = self.ik.forward(q)
        return pos

    def step(
        self,
        q: np.ndarray,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
        dt: float,
        max_joint_step: float | None = None,
    ) -> DiffIkStep:
        """单步积分：v_cmd = v_des + Kp * e_pos, dq = J^+ v_cmd."""
        pin.framesForwardKinematics(self.model, self.data, q)
        ee_pos = self.data.oMf[self.frame_id].translation
        pos_err = target_position - ee_pos
        v_cmd = target_velocity + self.pos_gain * pos_err

        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.frame_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        lhs = J @ J.T + self.damp * np.eye(3)
        dq = J.T @ np.linalg.solve(lhs, v_cmd)
        dq = np.clip(dq, -self.max_dq, self.max_dq)

        delta = pin.integrate(self.model, q, dq * dt) - q
        if max_joint_step is not None:
            delta = np.clip(delta, -max_joint_step, max_joint_step)
        q_next = pin.integrate(self.model, q, delta)
        q_next = np.clip(q_next, self.q_lower, self.q_upper)
        dq = delta / dt
        pos_err_norm = float(np.linalg.norm(pos_err))
        return DiffIkStep(
            q=q_next,
            dq=dq,
            position_error=pos_err_norm,
            success=pos_err_norm < 0.01,
        )


class PrecomputedCircleTrajectory:
    """离线沿圆轨迹解 IK，运行时查表插值，零在线 IK 开销。"""

    def __init__(
        self,
        center: np.ndarray,
        radius: float,
        period_sec: float,
        sample_hz: float = 200.0,
        plane: str = 'yz',
    ):
        self.center = np.asarray(center, dtype=float)
        self.radius = radius
        self.period_sec = period_sec
        self.omega = 2.0 * np.pi / period_sec
        self.dt = 1.0 / sample_hz
        self.plane = plane
        self.ik = PinocchioIK()
        self._build_table()

    def _offset(self, phase: float) -> np.ndarray:
        c, s = np.cos(phase), np.sin(phase)
        if self.plane == 'yz':
            return np.array([0.0, c, s])
        if self.plane == 'xy':
            return np.array([c, s, 0.0])
        raise ValueError(f'unsupported plane: {self.plane}')

    def _build_table(self) -> None:
        n = max(2, int(round(self.period_sec / self.dt)))
        self.times = np.linspace(0.0, self.period_sec, n, endpoint=False)
        self.q_table = np.zeros((n, self.ik.nq))
        q_last, ok = self.ik.solve(self.center + self.radius * self._offset(0.0))
        if not ok:
            raise RuntimeError('circle start pose unreachable')
        self.q_table[0] = q_last
        for i in range(1, n):
            phase = self.omega * self.times[i]
            target = self.center + self.radius * self._offset(phase)
            q, ok = self.ik.solve(target, q_init=q_last)
            if not ok:
                raise RuntimeError(f'IK failed at sample {i}/{n}')
            self.q_table[i] = q
            q_last = q
        dq = np.diff(self.q_table, axis=0, append=self.q_table[:1]) / self.dt
        self.dq_table = dq

    def sample(
        self,
        t: float,
        q_prev: np.ndarray | None = None,
        max_joint_step: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        t = t % self.period_sec
        idx_f = t / self.dt
        i0 = int(idx_f) % len(self.times)
        i1 = (i0 + 1) % len(self.times)
        alpha = idx_f - int(idx_f)
        q = (1.0 - alpha) * self.q_table[i0] + alpha * self.q_table[i1]
        dq = (1.0 - alpha) * self.dq_table[i0] + alpha * self.dq_table[i1]
        if q_prev is not None and max_joint_step is not None:
            delta = np.clip(q - q_prev, -max_joint_step, max_joint_step)
            q = q_prev + delta
            dq = delta / max(self.dt, 1e-6)
        return q, dq
