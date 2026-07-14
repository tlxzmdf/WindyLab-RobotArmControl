# 真主臂 → 仿真从臂 + 末端自稳

## 两种用法

| 模式 | 脚本 | 机载端 | 从臂控制 | link4 突变 |
|------|------|--------|----------|------------|
| **A 固定** | `./run_hw.sh` | 不动 | `joint_mirror` 直接复制主臂关节 | **已消除** |
| **B 扰动** | `./run_hw_disturbed.sh` | 随机 Roll/Pitch/Yaw | `ee_stabilization` IK + 步长限制 | 可能有小幅差异 |

仿真对应：`./run_sim.sh` / `./run_sim_disturbed.sh`

## 快速开始

```bash
cd /root/arm/projects/master-slave-stabilization

# 模式 A：机载端固定（台架 / 手拖主臂，从臂 1:1 跟随关节）
PORT_NAME=/dev/ttyUSB0 ./run_hw.sh

# 模式 B：机载端随机扰动（世界系末端自稳）
PORT_NAME=/dev/ttyUSB0 ./run_hw_disturbed.sh
```

## 原理

**模式 A — `teleop_control_mode: joint_mirror`**

机载端固定时，主从臂基座相同，无需 IK。从臂直接复制 `/master/joint_states`，彻底避免腕部冗余导致的 link4 180° 跳变。

**模式 B — `teleop_control_mode: ee_stabilization`**

机载端扰动时使用 **CLIK（闭环逆运动学）** + **Liegeois 零空间投影**，以上一时刻关节构型为参考，在完成任务的同时保持关节轨迹连续；在 `|q5|→0` 腕部奇异附近自动增大阻尼（DLS）。不再使用关节步长截断。

## 配置文件

| 文件 | 模式 |
|------|------|
| `config/teleop_stabilization.yaml` | A：固定机载端 |
| `config/teleop_stabilization_disturbed.yaml` | B：随机扰动 |

## 核心话题

| 话题 | 说明 |
|------|------|
| `/master/joint_states` | 主臂关节 |
| `/joint_states` | 从臂仿真 |
| `/stabilization_markers` | RViz 可视化 |

## 真机主臂轻微自主运动

零力模式下 `GravityController` 持续输出重力补偿力矩，URDF 与真机不一致时会有轻微漂移。诊断：

```bash
cd scripts && python3 diagnose_master_drift.py
```
