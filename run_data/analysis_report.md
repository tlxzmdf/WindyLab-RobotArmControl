# 机械臂运行误差分析报告

| 运行 | Demo | max_vel | EE RMS (mm) | 路径 RMS (mm) | 饱和率 | joint1 RMS |
|------|------|---------|-------------|---------------|--------|------------|
| 165443_move_arm_demo | move_arm_demo.py | 0.20 | 64.0 | — | 30.6% | 0.088 |
| 170158_move_arm_ik_demo | move_arm_ik_demo.py | 0.20 | 46.1 | — | 5.0% | 0.104 |
| 170230_move_arm_line_demo | move_arm_line_demo.py | 0.20 | 156.2 | — | 17.4% | 0.653 |
| 170253_rotate_link5_right_90 | rotate_link5_right_90.py | 0.20 | 79.6 | — | 9.5% | 0.003 |
| 170318_move_arm_demo | move_arm_demo.py | 0.20 | 202.6 | — | 30.7% | 0.267 |
| 170654_circle_draw_diff | circle_draw_node.py --mode d | 0.35 | 29.1 | 56.0 | 0.0% | 0.113 |
| 170731_circle_draw_precompute | circle_draw_node.py --mode p | 0.35 | 66.0 | 85.5 | 0.0% | 0.359 |

## 误差原因与避免方法

### 1. 底层限速（最主要）

`SmoothPositionController` 每 10 ms 将关节位置增量限制在 `±max_velocity × dt`。
若规划器单步所需速度超过 `max_velocity`，真机必然滞后，表现为关节/末端误差累积。

**避免：**
- 画圆/快速轨迹：`ARM_MAX_VELOCITY=0.35` 启动 launch（与 `circle-draw` 规划一致）
- 规划侧限制单步：`max_joint_step = max_velocity × dt`（`circle_draw_node` 已做）
- 离线检查饱和率：本工具 `cmd_saturation_pct`，目标 < 10%

### 2. 指令频率与控制器周期不匹配

`student_arm_node` 控制周期 100 Hz；`move_arm_ik_demo` 仅 50 Hz 且无速度前馈。

**避免：**
- 用 `circle-draw`（100 Hz）替代原版 `move_arm_ik_demo`
- 模式选 `diff`（在线微分 IK + 速度前馈）

### 3. 速度前馈未被位置控制器使用

当前 `SmoothPositionController` **只跟踪位置 setpoint**，`JointState.velocity` 写入 setpoint.dq 但未参与计算。
因此发布 `velocity` 对真机帮助有限，主要仍靠位置追赶。

**避免（进阶）：**
- 仿真验证可开 `kinematic_mode:=True`（无限速）
- 或扩展控制器使用 `setpoint.dq` 前馈（需改 C++）

### 4. 位置 IK 帧间跳变

原版 demo 每帧独立迭代 IK，关节增量偶发超限。

**避免：**
- 微分 IK + 热启动（`circle-draw --mode diff`）
- 或 `precompute` 离线轨迹查表

### 5. 录制与启动时序

Demo 应在收到 `/joint_states` 后再发指令；`rotate_link5` 已等待反馈。

**避免：**
- 终端 1 launch 稳定 2 s 后再启 Demo
- 使用 `record_hw.sh` / `pc_arm_record_demo.sh` 自动等待话题

### 6. 本次录制对比（数据结论）

- **最佳画圆**：`20260626_170654_circle_draw_diff` — 路径 RMS **56.0 mm**
- **较差画圆**：`20260626_170731_circle_draw_precompute` — 路径 RMS **85.5 mm**
- 推荐真机画圆：`circle-draw --mode diff`，`max_velocity=0.35`

- 原版 `move_arm_ik_demo` 末端 RMS 46.1 mm → `circle-draw diff` 路径 RMS 56.0 mm

## 20260626_165443_move_arm_demo

- Demo: `move_arm_demo.py`
- max_joint_velocity: 0.2 rad/s
- 末端 FK RMS: 64.04 mm
- 指令饱和率: 30.6%
- 备注: 指令关节速度饱和率 30.6%（>0.2 rad/s）

## 20260626_170158_move_arm_ik_demo

- Demo: `move_arm_ik_demo.py`
- max_joint_velocity: 0.2 rad/s
- 末端 FK RMS: 46.05 mm
- 指令饱和率: 5.0%

## 20260626_170230_move_arm_line_demo

- Demo: `move_arm_line_demo.py`
- max_joint_velocity: 0.2 rad/s
- 末端 FK RMS: 156.22 mm
- 指令饱和率: 17.4%
- 备注: 指令关节速度饱和率 17.4%（>0.2 rad/s）

## 20260626_170253_rotate_link5_right_90

- Demo: `rotate_link5_right_90.py`
- max_joint_velocity: 0.2 rad/s
- 末端 FK RMS: 79.58 mm
- 指令饱和率: 9.5%

## 20260626_170318_move_arm_demo

- Demo: `move_arm_demo.py`
- max_joint_velocity: 0.2 rad/s
- 末端 FK RMS: 202.57 mm
- 指令饱和率: 30.7%
- 备注: 指令关节速度饱和率 30.7%（>0.2 rad/s）

## 20260626_170654_circle_draw_diff

- Demo: `circle_draw_node.py --mode diff`
- max_joint_velocity: 0.35 rad/s
- 末端 FK RMS: 29.10 mm
- 理想轨迹 RMS: 55.98 mm
- 指令饱和率: 0.0%
- 备注: 末端偏离理想轨迹 RMS 56.0 mm

## 20260626_170731_circle_draw_precompute

- Demo: `circle_draw_node.py --mode precompute`
- max_joint_velocity: 0.35 rad/s
- 末端 FK RMS: 66.02 mm
- 理想轨迹 RMS: 85.47 mm
- 指令饱和率: 0.0%
- 备注: 末端偏离理想轨迹 RMS 85.5 mm
