# master-slave-wbc

真主臂遥操作 + 仿真从臂，控制核心为 **QP 速度 WBC + 积分动作**（替代原 `master-slave-stabilization` 中的 CLIK/IK 链路）。

## 架构

| 组件 | 包/路径 |
|------|---------|
| 控制节点 | `windylab_ws/src/arm_teleop_wbc_control` → `teleop_wbc` |
| 可视化 URDF | `arm_ee_stabilization_description` |
| 项目配置/启动 | 本目录 `config/`、`launch/`、`scripts/` |

### 控制律（模式 B）

1. 主臂 FK → 滤波得到世界系末端目标 `ee_target_world`
2. 机载端扰动 → 目标变换到基座系 `T_des_base`
3. 任务误差 + 机载端前馈速度 → **PI 积分动作** 得到期望任务速度 `v_cmd`
4. 带盒约束 QP 求关节速度，零空间参考为主臂关节角
5. 多子步积分关节角

## 两种模式

| 模式 | 配置 | 机载端 | 从臂控制 |
|------|------|--------|----------|
| **A** | `teleop_wbc.yaml` | 固定 `base_source: static` | `joint_mirror` 直接关节映射 |
| **B** | `teleop_wbc_disturbed.yaml` | 随机/脚本扰动 | `ee_wbc` QP-WBC + 积分 |

## 快速启动

```bash
cd /root/arm/projects/master-slave-wbc

# 模式 A 仿真
./run_sim.sh

# 模式 B 仿真（机载端扰动）
./run_sim_disturbed.sh

# 模式 A 真机主臂
PORT_NAME=/dev/ttyUSB0 ./run_hw.sh

# 模式 B 真机主臂 + 扰动
PORT_NAME=/dev/ttyUSB0 ./run_hw_disturbed.sh
```

## 主要参数

见 `config/teleop_wbc_disturbed.yaml`：

- `wbc_task_kp` / `wbc_task_ki`：任务 PI（含积分动作）
- `wbc_nullspace_weight`：主臂关节零空间参考
- `wbc_substeps`：每控制周期 QP 积分子步数
- `max_joint_velocity`：QP 关节速度盒约束

## 数据采集与回归

```bash
./scripts/run_experiments.sh       # Mode A + B，各 25s CSV
./scripts/run_mode_b_benchmark.sh  # 仅 Mode B 快速验证
./scripts/run_clik_baseline.sh     # CLIK 对照
python3 scripts/summarize_reports.py
```

报告输出在 `reports/EXPERIMENT_SUMMARY.md`。Mode B 修复后 world EE 位置 RMS ~2.7 mm（CLIK 对照 ~2.3 mm），无 ≥46° 的 joint4 阶跃跳变。

- 输入：`/master/joint_states`
- 输出：`/joint_states`（mount + arm）
- 调试：`/teleop_wbc_error`、`/teleop_wbc_markers`
