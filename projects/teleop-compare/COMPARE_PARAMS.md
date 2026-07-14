# Mode B 对比参数说明

## 参数对照表（统一前 → 统一后）

| 参数 | 含义 | stabilization 原值 | WBC 原值 | **统一对比值** | 说明 |
|------|------|-------------------|----------|----------------|------|
| `control_rate` | 控制频率 Hz | 500 | 500 | **500** | 一致 |
| `teleop_target_filter` | 末端目标滤波 | 0.12 | 0.12 | **0.12** | 一致 |
| `substeps` | 每周期子步数 | 10 (`teleop_clik_substeps`) | 8→10 (`wbc_substeps`) | **10** | WBC 默认 8 偏弱，已提升 |
| `task_kp` | 任务 P 增益 | [32,32,32,24,24,24] | 默认 [24..18] | **[32,32,32,24,24,24]** | WBC 默认 yaml 偏弱，已提升 |
| `nullspace` | 零空间增益 | 0.25 | 0.2→0.25 | **0.25** | WBC 默认 0.2 偏弱 |
| `clik_damping` | 阻尼 λ | 0.035 | 0.035 | **0.035** | 一致 |
| `max_joint_velocity` | 关节速度上限 rad/s | 4.5 | 4.0→4.5 | **4.5** | WBC 默认 4.0 偏弱 |
| `disturbance_radius` | 扰动球半径 m | 0.08 | 0.12→0.08 | **0.08** | WBC 默认 0.12 不一致 |
| `disturbance_orient_amp` | 姿态扰动 rad | 0.262 | 0.18→0.262 | **0.262** | 已对齐 |
| `disturbance_time_constant` | 扰动时间常数 s | 2.5 | 1.0→2.5 | **2.5** | 已对齐 |
| `disturbance_amplitude_scale` | 扰动幅度比 | 0.90 | 0.92→0.90 | **0.90** | 已对齐 |

## 方案特有参数（对比时保留，不强行统一）

| 参数 | 方案 | 值 | 说明 |
|------|------|-----|------|
| `teleop_wrist_singularity_damping_scale` | CLIK | 0.04 | 腕部奇异阻尼，WBC 无对应项 |
| `wbc_task_ki` | WBC | [8,8,8,6,6,6] | 积分动作，CLIK 无 |
| `wbc_nullspace_rate` | WBC | 4.0 | 零空间速率 |
| `q_des_filter` | CLIK | 0.55 | kinematic 模式下实际未滤波 |

## 配置文件

统一对比请使用：

- CLIK：`master-slave-stabilization/config/teleop_compare_mode_b.yaml`
- WBC：`master-slave-wbc/config/teleop_compare_mode_b.yaml`

（内容与 `teleop-compare/config/teleop_compare_mode_b.yaml` 同步）
