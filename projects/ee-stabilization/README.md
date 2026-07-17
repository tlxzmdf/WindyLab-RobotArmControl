# 机械臂末端位姿稳定 (EE Stabilization)

基于 **A-L1-GAMMA** 机械臂与 Pinocchio 动力学，实现 **机头稳定**：无人机与机械臂连接处（`drone_mount`）在球内随机扰动，末端在世界坐标系中的 **位置与方向保持固定**。

## 本目录结构

```text
ee-stabilization/
├── README.md           # 本文件（快速上手）
├── run_sim.sh          # RViz 仿真
├── run_hw.sh           # 真机三模式 A/B/C
├── docs/
│   ├── main.tex        # 项目技术说明（LaTeX 源文件）
│   ├── 1.png           # 控制框图
│   └── pdf/            # 编译输出的 PDF
├── scripts/
│   ├── test_all_modes.py      # 三模式自动化测试
│   ├── run_three_mode_test.py # 备用测试脚本
│   ├── build_doc_pdf.sh       # main.tex → PDF
│   └── md_to_pdf.sh           # README → PDF
└── reports/            # 测试输出（曲线、指标表）
```

**ROS 包路径：** `../../windylab_ws/src/arm_ee_stabilization_{description,control}`  
**参数配置：** `../../windylab_ws/src/arm_ee_stabilization_control/config/stabilization.yaml`

## 场景

```text
world
  └─ drone_mount  ← 在球内随机平移 + 小角度姿态扰动 (模拟无人机晃动)
       └─ base_link → … → link6 (末端)
```

启动时锁定当前末端位姿为世界系目标，之后无论 mount 如何运动，控制器通过 IK / 计算力矩 / OSC 调整关节，使末端回到目标。

## 启动

```bash
# RViz 仿真
./run_sim.sh

# 或从工作区根目录（兼容旧路径）
/root/arm/run_ee_stabilization.sh
```

手动 launch：

```bash
cd /root/arm/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch arm_ee_stabilization_description stabilization.launch.py
```

## 三模式自动化测试

```bash
cd /root/arm/projects/ee-stabilization
source /opt/ros/humble/setup.bash
source ../../windylab_ws/install/setup.bash
python3 scripts/test_all_modes.py
```

报告输出至 `reports/mode_test_report.md`。测试在**规划+控制完整闭环**下运行（非开环规划）：

| 图表 | 评价内容 |
|------|----------|
| `mode_{A,B,C}_planning.png` | 末端位姿（world→link6）：规划+控制+动力学后的任务空间效果 |
| `mode_{A,B,C}_control.png` | 六关节 $q_{act}$ vs $q_{cmd}$：控制层关节跟踪（模式 C 的 $q_{cmd}$ 为 IK 遥测） |

底层执行：仿真为 Pinocchio ABA 积分；真机为 `ee_stabilization` → `/student/joint_command` → `student_arm_node`（MIT 阻抗 + 力矩前馈）。

## RViz 可视化

| 元素 | 含义 |
|------|------|
| 黄色半透明球 | 连接点扰动范围 |
| 绿色透明球 | 锁定的目标末端（世界系） |
| 橙色实心球 | 实际末端 |
| 红色短线 | 目标与实际误差 |
| 橙色 mount 球 | 无人机连接点 |

## 真机运行（三模式）

仿真与真机共用算法；真机通过 `hardware_mode` 发布 `/student/joint_command`，由 `student_arm_node` 以 **MIT 协议**驱动电机。

| 模式 | 真机实现 | 说明 |
|------|----------|------|
| **A** | IK → MIT **位置**跟踪 | `effort=0`，推荐首次上真机 |
| **B** | IK 位置 + **计算力矩**前馈 | `effort=τ_ctc` |
| **C** | 实测位置 + **OSC 力矩**前馈 | `effort=τ_osc` |

```bash
./run_hw.sh A          # 模式 A（默认）
./run_hw.sh B
./run_hw.sh C

# 环境变量
ARM_TYPE=a_l1 PORT_NAME=/dev/ttyTHS3 BASE_SOURCE=simulated ./run_hw.sh A

# 上真机前用仿真臂验证 MIT 链路
ARM_TYPE=sim ./run_hw.sh A
```

完整 launch：

```bash
cd /root/arm/windylab_ws && source install/setup.bash
ros2 launch arm_ee_stabilization_description stabilization_hardware.launch.py \
  stabilization_mode:=A arm_type:=a_l1 port_name:=/dev/ttyTHS3 \
  base_source:=simulated use_rviz:=False
```

### 力矩前馈接口（真机）

真机 launch 会同时拉起：

1. `student_arm_node`，参数来自 `manipulator/stabilization_hw_student_arm.yaml`：`controller_type: mit_stabilization`、`torque_limit: 9.0`
2. `ee_stabilization`（`hardware_mode:=true`），向 `/student/joint_command` 发布 `sensor_msgs/JointState`

消息字段约定：

| 字段 | 含义 |
|------|------|
| `position` | MIT 目标关节角 (rad)；模式 C 钉住实测 `q` |
| `velocity` | MIT 速度通道；模式 C 可经 `hw_zero_dq` 置 0 |
| `effort` | 力矩前馈 τ (Nm) → MIT `current` 通道（按力矩使用） |

`effort` 是否非零由模式决定：A 关闭前馈；B/C（及 D）开启。限幅：

| 层 | 参数 | A/B | C |
|----|------|-----|---|
| 稳定节点 | `hw_torque_limit` | 9.0 | **6.0** |
| MIT 控制器 | `torque_limit` | 9.0 | 9.0 |
| 电机 | `rated_torque` | 近端 9.0 / 远端 1.6 | 同左 |

这是 **MIT 阻抗 + 力矩前馈**，不是纯力矩控制。普通 `student_arm.launch.py` 默认 `smooth`，自行发 `effort` 不会生效；见仓库根目录 `STUDENT_GUIDE.md` / `详细使用手册.md` §3.4。

### 基座来源 (`base_source`)

| 值 | 用途 |
|---|---|
| `simulated` | 软件模拟 UAV 晃动（台架测试） |
| `static` | 基座固定 |
| `tf` | 从 TF `world → base_link` 读取真实基座 |

## 话题

| 话题 | 说明 |
|------|------|
| `/joint_states` | 关节状态 |
| `/stabilization_markers` | RViz Marker |
| `/stabilization_error` | 末端误差 |
| TF `world` → `drone_mount` | 连接点位姿 |

## 文档

- 技术说明：[`docs/main.tex`](docs/main.tex)（Design Spec：设计目标/原则 → 架构 → 数学模型 → 控制 → 执行 → 实验 → 附录）
- 简化版：[`docs/main_brief.tex`](docs/main_brief.tex)（要点摘要，完整版见上）
- 编译 PDF：`./scripts/build_doc_pdf.sh` → `docs/pdf/项目说明.pdf`
- 编译简化版：`./scripts/build_doc_brief_pdf.sh` → `docs/pdf/项目说明-简化版.pdf`
- README PDF：`./scripts/md_to_pdf.sh` → `docs/pdf/README.pdf`
