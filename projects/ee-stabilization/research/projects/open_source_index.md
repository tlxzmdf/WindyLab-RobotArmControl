# 末端稳定相关开源项目索引

> 调研日期：2026-07-08  
> 与本项目 `ee-stabilization` 的相关性按 ★（低）～★★★★★（高）标注。

| 项目 | 语言/框架 | Stars | 核心能力 | 与本项目关系 | 链接 |
|------|-----------|-------|----------|--------------|------|
| **ManipulationOnTheMove** | ROS + Unity 仿真 | — | 移动基座扰动下反应式末端世界系稳定 | ★★★★★ 场景几乎一致：无先验基座运动、靠观测补偿 gripper | [GitHub](https://github.com/BenBurgessLimerick/ManipulationOnTheMove) |
| **MotM-BaseControl** | ROS | 12 | 移动操作基座反应控制 | ★★★★ 基座+臂协同，但可借鉴反应式架构 | [GitHub](https://github.com/BenBurgessLimerick/MotM-BaseControl) |
| **StanfordASL/oscbf** | Python | 205 | 操作空间控制 + CBF 安全滤波 | ★★★★ 支持移动基座（PPR 链首），kHz OSC | [GitHub](https://github.com/StanfordASL/oscbf) |
| **ARC-OPT/wbc_ros** | ROS2 Humble | — | 全身 WBC 库 ROS2 接口 | ★★★★ 可扩展为浮动基座 QP 任务栈 | [GitHub](https://github.com/ARC-OPT/wbc_ros) |
| **learnsyslab/upright** | ROS + OCS2 MPC | 148 | 移动臂非抓取物体平衡（托盘问题） | ★★★ 任务不同，但 OCS2 约束建模可参考 | [GitHub](https://github.com/learnsyslab/upright) |
| **SYSU-HILAB/am-planner** | ROS Noetic + PyTorch | — | 空中机械臂全身轨迹规划 | ★★★★ 规划层 whole-body，偏轨迹而非定点稳定 | [GitHub](https://github.com/SYSU-HILAB/am-planner) |
| **utiasDSL/mobile_manipulation_central** | ROS | — | Ridgeback+UR10 移动操作硬件栈 | ★★★ 移动操作基础设施 | [GitHub](https://github.com/utiasDSL/mobile_manipulation_central) |
| **Isaac Lab OSC** | Python/NVIDIA | — | 浮动基座 OSC（需注意索引偏移 bug） | ★★★ 仿真验证 OSC 浮动基座实现细节 | [Issue #4999](https://github.com/isaac-sim/IsaacLab/issues/4999) |
| **arm-platform (本仓库)** | ROS2 + Pinocchio | — | A/B/C 三模式 EE 稳定 | ★★★★★ 被调研对象 | `../../windylab_ws/src/arm_ee_stabilization_*` |

## 按技术路线分类

### 1. 纯运动学 / CLIK / IK
- 本项目的 **模式 A** 与文献中 CLIK、DLS IK 路线一致。
- `arm-platform/demo/pinocchio_ik.py` 与 C++ 求解器对齐。

### 2. 操作空间控制 (OSC)
- **oscbf**：现代 Python OSC + 安全约束，支持移动基座链。
- 本项目 **模式 C**：Pinocchio 动力学 + 任务空间力矩。

### 3. 全身二次规划 (Whole-Body QP)
- **wbc_ros** + **am-planner**：多任务优先级、约束显式处理。
- 文献 RAL 2019 力反馈 QP、arXiv 2601.08523 欠驱动 AM QP。

### 4. 反应式 / 无模型基座补偿
- **ManipulationOnTheMove**：控制器不知基座运动先验，仅靠观测稳定 gripper——与真机 `base_source=tf` 场景接近。

### 5. MPC / 轨迹优化
- **upright (OCS2)**：适合长时域、约束丰富任务；定点稳定通常 overkill。

## 可借鉴的实现要点

1. **MotM / ManipulationOnTheMove**：分层反应架构、失败恢复时的基座-臂协同。
2. **oscbf**：CBF 安全层可叠加在现有 OSC 输出上，处理关节限位/奇异。
3. **wbc_ros**：若未来需要 UAV+臂联合控制，可直接接入 QP 任务栈。
4. **Isaac Lab**：浮动基座 OSC 的 `mass_matrix` 索引偏移问题——移植到 Pinocchio 时需注意基座 DOF 不参与 `M,h` 提取。
