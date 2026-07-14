#!/usr/bin/env python3
"""诊断真机主臂轻微自主运动 / 漂移。

在 run_hw.sh 运行时另开终端执行:
  python3 diagnose_master_drift.py

原理: 主臂 MIT 零刚度模式下 τ_motor = τ_gravity(URDF)。
若模型与真机不完全一致，会产生净力矩 → 缓慢漂移。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF = PROJECT_ROOT / 'urdf' / 'arm_link7_zero_mass.urdf'
PLATFORM_URDF = PROJECT_ROOT.parents[1] / 'windylab_ws' / 'src' / 'arm-platform' / 'config' / 'arm.urdf'
JOINTS = [f'joint{i}' for i in range(1, 8)]


class MasterDriftDiag(Node):
    def __init__(self) -> None:
        super().__init__('master_drift_diag')
        urdf_path = URDF if URDF.is_file() else PLATFORM_URDF
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self._last_q: np.ndarray | None = None
        self._last_t: float | None = None
        self._samples = 0
        self.create_subscription(JointState, '/master/joint_states', self._cb, 50)
        self.create_timer(2.0, self._report)
        self.get_logger().info('监听 /master/joint_states，每 2s 输出诊断…')

    def _extract(self, msg: JointState) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        q = np.zeros(7)
        dq = np.zeros(7)
        tau_meas = np.zeros(7)
        got = 0
        for i, name in enumerate(JOINTS):
            if name not in msg.name:
                continue
            idx = msg.name.index(name)
            if idx < len(msg.position):
                q[i] = msg.position[idx]
            if idx < len(msg.velocity):
                dq[i] = msg.velocity[idx]
            if idx < len(msg.effort):
                tau_meas[i] = msg.effort[idx]
            got += 1
        return (q, dq, tau_meas) if got == 7 else None

    def _cb(self, msg: JointState) -> None:
        parsed = self._extract(msg)
        if parsed is None:
            return
        q, dq, tau_meas = parsed
        pin.computeGeneralizedGravity(self.model, self.data, q)
        tau_model = self.data.g.copy()
        self._last_q = q
        self._last_dq = dq
        self._last_tau_model = tau_model
        self._last_tau_meas = tau_meas
        self._samples += 1

    def _report(self) -> None:
        if self._last_q is None:
            self.get_logger().warn('尚无 /master/joint_states 数据')
            return
        q = self._last_q
        dq = self._last_dq
        tau_m = self._last_tau_model
        tau_meas = self._last_tau_meas
        speed = float(np.linalg.norm(dq))
        max_dq = float(np.max(np.abs(dq)))
        max_q = float(np.max(np.abs(q)))
        # 模型重力矩 vs 实测电流(近似力矩)
        tau_err = tau_m - tau_meas
        print(
            f'\n--- 样本={self._samples} ---\n'
            f'关节速度: |dq|={speed:.4f} rad/s  max|dq|={max_dq:.4f} rad/s\n'
            f'Pinocchio 重力矩 τ_model (Nm): '
            f'{", ".join(f"j{i+1}:{tau_m[i]:+.2f}" for i in range(7))}\n'
            f'实测 effort/电流 (MotorState): '
            f'{", ".join(f"j{i+1}:{tau_meas[i]:+.2f}" for i in range(7))}\n'
            f'τ_model - τ_meas (Nm): '
            f'{", ".join(f"j{i+1}:{tau_err[i]:+.2f}" for i in range(7))}',
            flush=True,
        )
        if speed > 0.02:
            print(
                '  → 检测到明显运动: 常见原因 = 重力补偿过/不足(URDF≠真机)、'
                'kd=0 无阻尼、或关节在限位附近。',
                flush=True,
            )
        elif speed > 0.005:
            print(
                '  → 轻微漂移: 正常范围内，主因是 τ_gravity 连续输出 + 模型误差。',
                flush=True,
            )
        else:
            print('  → 几乎静止: 重力补偿与当前姿态较匹配。', flush=True)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = MasterDriftDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
