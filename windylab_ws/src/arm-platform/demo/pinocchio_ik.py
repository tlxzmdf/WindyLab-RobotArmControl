#!/usr/bin/env python3
"""基于 Pinocchio 的逆运动学 (IK) 模块。

针对 A-L1-GAMMA 7 自由度机械臂 (src/arm-platform/config/arm.urdf)。
采用阻尼最小二乘 (Damped Least Squares / Levenberg-Marquardt) 迭代求解。

说明:
- 该臂 joint6 与 joint7 轴线共线 (均为 x 轴), 末端姿态存在病态方向,
  因此默认只做 3D 位置 IK (7 关节冗余, 求解稳定);
  如需姿态, 可传入 target_rotation, 姿态误差以较低权重加入任务。
- 求解结果自动钳制到 URDF 关节限位内。

用法示例:
    from pinocchio_ik import PinocchioIK
    ik = PinocchioIK()
    q = ik.solve(np.array([0.4, 0.0, 0.2]))        # 仅位置
    pos, rot = ik.forward(q)                        # 正解验证
"""

import os
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

def _resolve_default_urdf() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(here)
    for candidate in (
        os.path.join(pkg_root, 'config', 'arm.urdf'),
        os.path.join(pkg_root, 'arm.urdf'),
        os.path.join(os.path.dirname(pkg_root), 'config', 'arm.urdf'),
    ):
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f'arm.urdf not found near {pkg_root}')


DEFAULT_URDF = _resolve_default_urdf()

EE_FRAME = 'link7'  # 末端 frame 名


@dataclass
class IkResult:
    q: np.ndarray
    position_error: float
    orientation_error: float
    success: bool
    acceptable: bool
    iterations: int


class PinocchioIK:
    def __init__(self, urdf_path: str = DEFAULT_URDF, ee_frame: str = EE_FRAME):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq
        # URDF 关节限位
        self.q_lower = self.model.lowerPositionLimit
        self.q_upper = self.model.upperPositionLimit

    def forward(self, q: np.ndarray):
        """正解: 返回末端 (position(3,), rotation(3,3))。"""
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def solve(self,
              target_position: np.ndarray,
              target_rotation: np.ndarray = None,
              q_init: np.ndarray = None,
              max_iter: int = 200,
              eps: float = 1e-4,
              dt: float = 0.5,
              damp: float = 1e-6,
              rot_weight: float = 0.1):
        """求解 IK。

        Args:
            target_position: 目标位置 (3,), base_link 坐标系。
            target_rotation: 可选目标姿态 (3,3) 旋转矩阵; None 则只解位置。
            q_init: 初值 (nq,); None 则用零位。重复调用时传上次解可热启动。
            max_iter / eps: 迭代上限与位置误差收敛阈值 (m)。
            dt: 迭代步长。
            damp: 阻尼系数 (避免奇异)。
            rot_weight: 姿态任务权重 (相对位置任务)。

        Returns:
            (q, ok): q 为关节角 (nq,) 已钳制限位; ok 表示是否收敛。
        """
        q = pin.neutral(self.model) if q_init is None else q_init.copy()

        for _ in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.frame_id]

            # 位置误差 (world 系)
            err_pos = target_position - oMf.translation
            if target_rotation is not None:
                err_rot = pin.log3(target_rotation @ oMf.rotation.T)
                err = np.concatenate([err_pos, rot_weight * err_rot])
            else:
                err = err_pos

            if np.linalg.norm(err_pos) < eps:
                return np.clip(q, self.q_lower, self.q_upper), True

            # 末端 frame 雅可比 (LOCAL_WORLD_ALIGNED)
            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                          pin.LOCAL_WORLD_ALIGNED)
            if target_rotation is not None:
                J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
            else:
                J = J6[:3, :]

            # 阻尼最小二乘: dq = J^T (J J^T + λI)^-1 err
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + damp * np.eye(JJt.shape[0]), err)
            q = pin.integrate(self.model, q, dq * dt)
            # 迭代中保持在限位内, 避免解跑飞
            q = np.clip(q, self.q_lower, self.q_upper)

        return q, np.linalg.norm(err_pos) < eps

    def solve_se3(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        q_init: np.ndarray = None,
        max_iters: int = 14,
        position_tol: float = 5e-4,
        orientation_tol: float = 8e-3,
        partial_pos_tol: float = 4e-3,
        partial_orient_tol: float = 5e-2,
        rot_weight: float = 1.0,
        damp: float = 0.012,
        step_scale: float = 0.9,
        max_step: float = 0.2,
    ) -> IkResult:
        """6D 位姿 IK (阻尼最小二乘), 与 arm-ee-stabilization C++ 求解器对齐."""
        q = pin.neutral(self.model) if q_init is None else q_init.copy()
        pos_err = float('inf')
        orient_err = float('inf')

        for it in range(max_iters):
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.frame_id]
            pos_err_vec = target_position - oMf.translation
            orient_err_vec = pin.log3(target_rotation @ oMf.rotation.T)
            pos_err = float(np.linalg.norm(pos_err_vec))
            orient_err = float(np.linalg.norm(orient_err_vec))

            success = pos_err < position_tol and orient_err < orientation_tol
            acceptable = pos_err < partial_pos_tol and orient_err < partial_orient_tol
            if success:
                q = np.clip(q, self.q_lower, self.q_upper)
                return IkResult(q, pos_err, orient_err, True, True, it + 1)

            err = np.concatenate([pos_err_vec, rot_weight * orient_err_vec])
            J6 = pin.computeFrameJacobian(
                self.model, self.data, q, self.frame_id, pin.LOCAL_WORLD_ALIGNED)
            J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
            lhs = J @ J.T + (damp * damp) * np.eye(6)
            dq = J.T @ np.linalg.solve(lhs, err)
            dq = np.clip(dq * step_scale, -max_step, max_step)
            q = pin.integrate(self.model, q, dq)
            q = np.clip(q, self.q_lower, self.q_upper)

        acceptable = pos_err < partial_pos_tol and orient_err < partial_orient_tol
        return IkResult(
            q, pos_err, orient_err,
            pos_err < position_tol and orient_err < orientation_tol,
            acceptable, max_iters)

    def solve_se3_multistage(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        q_init: np.ndarray = None,
        cycle_iters: int = 14,
        refine_iters: int = 22,
        recovery_iters: int = 36,
        **kwargs,
    ) -> IkResult:
        """多级 IK: 常规 -> refine -> recovery."""
        result = self.solve_se3(
            target_position, target_rotation, q_init=q_init,
            max_iters=cycle_iters, **kwargs)
        if result.acceptable:
            return result
        result = self.solve_se3(
            target_position, target_rotation, q_init=result.q,
            max_iters=refine_iters, **kwargs)
        if result.acceptable:
            return result
        return self.solve_se3(
            target_position, target_rotation, q_init=result.q,
            max_iters=recovery_iters, **kwargs)


if __name__ == '__main__':
    # 自测: 取一个可达位姿做 FK, 再用 IK 反求, 验证误差
    ik = PinocchioIK()
    print(f'模型: nq={ik.nq}, 末端 frame={EE_FRAME}')
    print(f'限位 lower: {np.round(ik.q_lower, 3)}')
    print(f'限位 upper: {np.round(ik.q_upper, 3)}')

    rng = np.random.default_rng(0)
    n_ok = 0
    for i in range(10):
        q_true = rng.uniform(ik.q_lower * 0.5, ik.q_upper * 0.5)
        p_target, _ = ik.forward(q_true)
        q_sol, ok = ik.solve(p_target)
        p_sol, _ = ik.forward(q_sol)
        e = np.linalg.norm(p_sol - p_target)
        n_ok += ok
        print(f'[{i}] target={np.round(p_target, 4)} err={e:.2e} ok={ok}')
    print(f'收敛 {n_ok}/10')
