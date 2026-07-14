# WindyLab 机械臂工作区

`/root/arm` 是 WindyLab 机械臂相关实验的共享工作区：平台驱动与教学 Demo 在 `windylab_ws/`，各应用/实验资产在 `projects/`。面向 **A-L1-GAMMA**（6/7 DoF）机械臂，环境为 **Ubuntu 22.04 + ROS 2 Humble**。

| 文档 | 用途 |
|------|------|
| [`STUDENT_GUIDE.md`](STUDENT_GUIDE.md) | 学生精简上手：仿真/真机、话题控制、IK |
| [`详细使用手册.md`](详细使用手册.md) | 完整手册：WSL2 串口、Demo 命令、排障 |
| `projects/<name>/README.md` | 各项目专项说明 |

仓库：https://github.com/tlxzmdf/WindyLab-RobotArmControl

---

## 目录结构

```text
arm/
├── README.md                      # 本文件：工作区总览
├── STUDENT_GUIDE.md               # 学生精简指南
├── 详细使用手册.md                 # 完整使用手册
├── .pc_arm_env.sh                 # 电脑/WSL 环境变量（本地，可忽略提交）
├── pc_real_arm_setup.sh           # 一键环境检测 / 编译 / 串口自检
├── pc_arm_launch*.sh              # 电脑版：真机 / 仿真 / RViz 启动
├── pc_arm_demo.sh                 # 运行 arm-platform/demo
├── pc_arm_record_demo.sh          # Demo + 数据录制 → run_data/
├── run_ee_stabilization*.sh       # 兼容旧路径 → projects/ee-stabilization/
├── run_data/                      # Demo 录制输出
├── projects/                      # 按项目隔离的文档 / 脚本 / 报告
│   ├── circle-draw/
│   ├── ee-stabilization/
│   ├── master-slave-stabilization/
│   ├── master-slave-wbc/
│   ├── teleop-compare/
│   └── reports/                   # 跨项目对比报告（可选）
└── windylab_ws/                   # 共享 colcon 工作空间
    ├── src/                       # 全部 ROS 包
    ├── tools/                     # 录制 / 分析 / 串口工具
    ├── build/  install/  log/     # 编译产物（gitignore）
    └── ...
```

---

## ROS 包一览（`windylab_ws/src/`）

| 包 | 说明 |
|----|------|
| [`arm-platform`](windylab_ws/src/arm-platform/)（ROS 名 `manipulator`） | 真机驱动、`student_arm_node`、MIT 协议、教学 Demo |
| [`dummy_description`](windylab_ws/src/dummy_description/) | 机械臂 / 云台等 URDF 与 mesh |
| [`dummy-interface`](windylab_ws/src/dummy-interface/) | 自定义消息（如 `MotorState`） |
| [`serial`](windylab_ws/src/serial/) | 串口通信依赖 |
| [`arm_ee_stabilization_description`](windylab_ws/src/arm_ee_stabilization_description/) | 机头稳定 launch / RViz / URDF 组合 |
| [`arm_ee_stabilization_control`](windylab_ws/src/arm_ee_stabilization_control/) | 机头稳定控制（IK / CTC / OSC 等模式） |
| [`arm_teleop_wbc_control`](windylab_ws/src/arm_teleop_wbc_control/) | 主从遥操作 QP-WBC 控制节点 |

平台相关 Python 工具在 [`windylab_ws/tools/`](windylab_ws/tools/)：`record_demo_run.py`、`plot_demo_run.py`、`analyze_run_data.py`、`serial_sniff.py`。

---

## 项目一览（`projects/`）

| 项目 | 目录 | 一句话说明 |
|------|------|------------|
| 末端画圆 | [`circle-draw/`](projects/circle-draw/) | 微分 IK / 预计算轨迹，解决画圆卡顿 |
| 机头稳定 | [`ee-stabilization/`](projects/ee-stabilization/) | UAV mount 扰动下末端世界系位姿保持 |
| 主从自稳 (CLIK) | [`master-slave-stabilization/`](projects/master-slave-stabilization/) | 真主臂 → 仿真从臂；关节镜像或 CLIK 末端自稳 |
| 主从自稳 (WBC) | [`master-slave-wbc/`](projects/master-slave-wbc/) | 同上场景，控制改为 QP-WBC + 积分动作 |
| CLIK vs WBC 对比 | [`teleop-compare/`](projects/teleop-compare/) | 公平参数下 CLIK / WBC 录制与对比 |

新建项目约定见下文；细节以各目录 `README.md` 为准。

### circle-draw — 末端画圆

针对 `arm-platform/demo/move_arm_ik_demo.py` 画圆卡顿：提高发布频率、微分 IK、关节速度前馈。

```bash
cd /root/arm/projects/circle-draw
./run_sim.sh                              # 仿真
ARM_MAX_VELOCITY=0.35 ./run_hw.sh         # 真机
./observe_hw.sh                           # 真机只读 RViz（不杀控制进程）
python3 scripts/benchmark_trajectory.py   # 离线对比
```

### ee-stabilization — 机头稳定

`drone_mount` 球内扰动，锁定末端世界系位姿；控制模式 A（IK 位置）/ B（计算力矩）/ C（OSC）。

```bash
cd /root/arm/projects/ee-stabilization
./run_sim.sh
./run_hw.sh A|B|C
python3 scripts/test_all_modes.py         # 三模式自动化测试 → reports/
```

ROS 包：`arm_ee_stabilization_{description,control}`。根目录 `run_ee_stabilization*.sh` 仅为旧路径转发。

### master-slave-stabilization — 主从 + CLIK

真主臂遥操作仿真从臂；模式 A 关节镜像，模式 B 机载端扰动 + CLIK / 零空间投影。

```bash
cd /root/arm/projects/master-slave-stabilization
./run_sim.sh / ./run_sim_disturbed.sh
PORT_NAME=/dev/ttyUSB0 ./run_hw.sh
PORT_NAME=/dev/ttyUSB0 ./run_hw_disturbed.sh
```

### master-slave-wbc — 主从 + QP-WBC

同场景，模式 B 用任务 PI + 盒约束 QP 求关节速度。包：`arm_teleop_wbc_control`。

```bash
cd /root/arm/projects/master-slave-wbc
./run_sim.sh / ./run_sim_disturbed.sh
PORT_NAME=/dev/ttyUSB0 ./run_hw.sh
PORT_NAME=/dev/ttyUSB0 ./run_hw_disturbed.sh
./scripts/run_experiments.sh              # 实验汇总 → reports/
```

### teleop-compare — CLIK vs WBC

统一参数见 [`COMPARE_PARAMS.md`](projects/teleop-compare/COMPARE_PARAMS.md)；推荐一次手拖、双方案回放对比。

```bash
cd /root/arm/projects/teleop-compare
PORT_NAME=/dev/ttyUSB0 DURATION=15 ./run_hw_compare_once.sh
DURATION=15 ./run_sim_compare.sh
```

---

## 电脑版常用脚本（工作区根目录）

在 WSL2 / 个人电脑上接真机时优先用这些脚本（串口默认 `/dev/ttyUSB0`，机载常用 `/dev/ttyTHS3`）。

| 脚本 | 作用 |
|------|------|
| `./pc_real_arm_setup.sh` | 检测依赖、编译、串口自检、打印启动指引 |
| `./pc_arm_launch_sim.sh` | 终端 1：仿真 `student_arm`（无 RViz） |
| `./pc_arm_launch.sh` | 终端 1：真机（无 RViz） |
| `./pc_arm_launch_rviz.sh` | 终端 1：真机 + RViz |
| `./pc_arm_demo.sh [demo.py]` | 运行 `arm-platform/demo`（默认 `move_arm_demo.py`） |
| `./pc_arm_record_demo.sh [demo] [dur] [sim\|a_l1]` | Demo + 录制到 `run_data/` |

常见 Demo：`move_arm_demo.py`、`move_arm_ik_demo.py`、`move_arm_line_demo.py`、`rotate_link5_right_90.py`。

---

## 编译与环境

```bash
cd /root/arm/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

新开终端需再次 `source /opt/ros/humble/setup.bash` 与 `source /root/arm/windylab_ws/install/setup.bash`。电脑版也可先跑 `./pc_real_arm_setup.sh`。

真机注意：先仿真验证；首次限速建议 `ARM_MAX_VELOCITY=0.2`；WSL 需用 `usbipd` 把 USB 串口 attach 进 Linux。详见 [`详细使用手册.md`](详细使用手册.md)。

---

## 新增项目约定

在 `projects/` 下新建子目录，建议包含：

- `README.md` — 快速上手
- `docs/` — 设计说明、PDF 等（可选）
- `scripts/` — 测试与工具
- `config/` / `launch/` — 若有项目级配置
- `reports/` — 自动化输出（目录内可 gitignore）
- `run_*.sh` — 仿真 / 真机入口

ROS 包仍放在 `windylab_ws/src/`，命名带项目前缀（如 `arm_ee_stabilization_*`、`arm_teleop_*`），避免冲突。并在本 README 与 [`projects/README.md`](projects/README.md) 中登记。
