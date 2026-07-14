# WindyLab 机械臂工作区

`/root/arm` 是多个机械臂相关项目的共享工作区。各项目的文档、脚本、测试报告放在 `projects/` 下；ROS 2 源码与编译产物统一放在 `windylab_ws/`。

## 目录结构

```text
arm/
├── README.md                 # 本文件：工作区总览
├── STUDENT_GUIDE.md          # 平台通用：学生上手机械臂
├── windylab_ws/                # 共享 colcon 工作空间（所有 ROS 包）
│   └── src/
│       ├── arm-platform/     # 真机驱动、MIT 控制
│       ├── arm_ee_stabilization_*  # 机头稳定（见下方项目）
│       └── ...
└── projects/                 # 按项目隔离的文档 / 脚本 / 报告
    ├── README.md
    └── ee-stabilization/     # 机头稳定
        ├── README.md
        ├── run_sim.sh
        ├── run_hw.sh
        ├── docs/
        ├── scripts/
        └── reports/
```

## 项目列表

| 项目 | 目录 | 说明 |
|------|------|------|
| 机头稳定 | [`projects/ee-stabilization/`](projects/ee-stabilization/) | 无人机 mount 扰动下末端世界系位姿保持 |
| *(待添加)* | `projects/<name>/` | 新项目请按相同结构创建子目录 |

## 新增项目约定

在 `projects/` 下创建独立子目录，例如 `projects/my-project/`：

- `README.md` — 快速上手
- `docs/` — 设计说明、PDF 等
- `scripts/` — 测试与工具脚本
- `reports/` — 自动化测试输出（可 gitignore）
- `run_*.sh` — 项目入口脚本（如需）

ROS 包仍放入 `windylab_ws/src/`，命名建议带项目前缀（如 `arm_ee_stabilization_*`），避免包名冲突。

## 常用命令

```bash
# 编译整个工作空间
cd /root/arm/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 机头稳定 — 仿真 RViz
/root/arm/projects/ee-stabilization/run_sim.sh

# 机头稳定 — 三模式自动化测试
cd /root/arm/projects/ee-stabilization
python3 scripts/test_all_modes.py
```

根目录下的 `run_ee_stabilization*.sh` 为兼容旧路径的转发脚本，新用法请直接进入 `projects/<项目名>/`。
