# 末端画圆轨迹优化 (circle-draw)

针对 `windylab_ws/src/arm-platform/demo/move_arm_ik_demo.py` 画圆卡顿问题的分析与优化实现。

## 卡顿原因

经 `scripts/benchmark_trajectory.py` 离线复现，**主要瓶颈不是 IK 计算慢**，而是**指令与底层控制不匹配**：

| 因素 | 原版 demo | 影响 |
|------|-----------|------|
| 发布频率 | 50 Hz | 低于 `student_arm_node` 100 Hz 控制周期，指令呈阶梯 |
| 关节速度 | 恒为 0 | 无速度前馈，`SmoothPositionController` 只能追位置 |
| 位置 IK | 每帧迭代收敛 | 单帧关节增量偶发 > `max_velocity × dt` |
| 限速 | 真机默认 0.2 rad/s | 约 23% 帧所需关节速度超限，末端滞后可达 ~20 mm |

原版在 50 Hz 下关节 5 峰值约 **0.30 rad/s**，超过 `.pc_arm_env.sh` 中 `ARM_MAX_VELOCITY=0.2`，控制器每 10 ms 最多移动 0.002 rad，长期累积即表现为**一顿一顿**。

## 优化方案

本目录提供两种模式（`circle_draw_node.py --mode`）：

1. **`diff`（默认）** — 100 Hz 微分 IK  
   - 解析圆轨迹线速度 + 位置误差反馈  
   - 单步 Jacobian，延迟低  
   - 发布 `JointState.velocity` 前馈  

2. **`precompute`** — 200 Hz 离线 IK 查表  
   - 启动时一次性解完整圈关节轨迹  
   - 运行时线性插值，零在线 IK  

## 目录结构

```text
circle-draw/
├── README.md
├── observe_hw.sh       # 真机只读观察（RViz，不干扰真机）
├── hw_observe.rviz     # 真机观察 RViz 配置
├── run_sim.sh          # RViz 仿真 + 优化画圆
├── run_hw.sh           # 真机画圆
└── scripts/
    ├── circle_draw_node.py      # ROS2 优化节点
    ├── ee_trajectory_viz.py     # 只读末端轨迹（observe_hw 用）
    ├── differential_ik.py       # 微分 IK / 预计算轨迹
    └── benchmark_trajectory.py  # 原版 vs 优化对比
```

## 真机运行时 RViz 观察（只读）

**与 `run_sim.sh` 互斥：** `run_sim.sh` 会 `pkill` 并启动 `arm_type:=sim`，会关掉真机。  
观察真机请用 **`observe_hw.sh`**：只订阅 `/joint_states` / TF，**不启动、不杀死**任何控制进程。

```bash
# 终端 1 — 真机（不变）
cd /root/arm && ARM_MAX_VELOCITY=0.35 ./pc_arm_launch.sh

# 终端 2 — 画圆或其它控制（不变）
cd /root/arm/projects/circle-draw/scripts
python3 circle_draw_node.py --mode diff --max-joint-velocity 0.35

# 终端 3 — 只读观察（新增）
cd /root/arm/projects/circle-draw
./observe_hw.sh
# 更快看到轨迹: TRAIL_DELAY=3 ./observe_hw.sh
# 仅 RViz 模型、不要轨迹线: WITH_TRAIL=0 ./observe_hw.sh
```

RViz 中：机械臂模型随真机关节同步；橙色小球为当前末端；橙色线为延迟轨迹（默认 10 s 后出现）。

## 快速开始

### 仿真

```bash
cd /root/arm/projects/circle-draw
./run_sim.sh
# 或: MODE=precompute ./run_sim.sh
```

### 真机

```bash
cd /root/arm/projects/circle-draw
./run_hw.sh
# 略提高限速（画圆需要 ~0.3 rad/s 峰值）:
ARM_MAX_VELOCITY=0.35 ./run_hw.sh
```

### 离线 benchmark

```bash
cd /root/arm/projects/circle-draw/scripts
python3 benchmark_trajectory.py
```

预期（`max_velocity = 0.2 rad/s` 仿真）：

| 方案 | 末端跟踪滞后 (均值/峰值) | 相对理想圆几何误差 (均值/峰值) |
|------|--------------------------|--------------------------------|
| 原版 50 Hz IK | 4.0 / 20.0 mm | 4.1 / 20.1 mm |
| 优化 diff IK | ~0 / ~0 mm | 1.4 / 6.8 mm |

## 参数

与原版 demo 一致，在 `circle_draw_node.py` 顶部修改：

- `CIRCLE_CENTER` — 圆心 (m)  
- `CIRCLE_RADIUS` — 半径 (m)  
- `PERIOD_SEC` — 周期 (s)  
- `PUBLISH_RATE_HZ` — 发布频率，默认 100  

## 与原版对比

| 项目 | `move_arm_ik_demo.py` | `circle_draw_node.py` |
|------|----------------------|------------------------|
| 频率 | 50 Hz | 100 Hz |
| IK | 迭代位置 IK (≤200 步) | 微分 IK 或预计算 |
| 速度前馈 | 无 | 有 |
| 适用场景 | 教学示例 | 平滑画圆 |

原版 demo 保留不变；本目录为独立项目，不修改 ROS 包。
