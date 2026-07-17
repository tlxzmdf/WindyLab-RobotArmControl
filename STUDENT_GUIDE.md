# 机械臂学生使用说明

本文档面向使用 A-L1-GAMMA 7 自由度机械臂做实验的学生。你将学会:启动仿真/真实机械臂、通过 ROS 2 话题控制关节、使用逆运动学 (IK) 控制末端位置。

说明：目前机械臂均为6自由度，如果末端有电机则为7自由度，但是该框架兼容二者。
---

## 1. 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- 系统依赖:

```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-rviz2 \
                 ros-humble-pinocchio libeigen3-dev
```

- Python 依赖(运行 IK 示例需要):

```bash
pip3 install numpy
# pinocchio 的 python 绑定随 ros-humble-pinocchio 一起安装
```

## 2. 获取与编译

工作空间包含 3 个机械臂相关的包:

| 包 | 作用 |
|---|---|
| `manipulator` (arm-platform) | 机械臂控制框架,含学生接口节点 |
| `dummy_interface` | 自定义消息 `MotorState`(关节反馈) |
| `dummy_description` | 机械臂 3D 模型 (RViz 可视化用) |

```bash
cd ~/windylab_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> 之后每开一个新终端,都要先执行:
> ```bash
> source /opt/ros/humble/setup.bash && source ~/windylab_ws/install/setup.bash
> ```

## 3. 启动机械臂

### 3.1 仿真模式(默认,推荐先用)

```bash
ros2 launch manipulator student_arm.launch.py
```

会启动:
- `student_arm_node` — 学生控制接口(虚拟臂)
- `robot_state_publisher` + RViz — 3D 可视化

### 3.2 真实硬件模式

**务必先在仿真中验证你的程序,确认无误后再上真机!**

**机载 NX（推荐快捷脚本）：**

```bash
cd ~/zihan_ws/arm            # 换成你的 arm 工作区路径
./nx_real_arm_setup.sh       # 首次一次
./nx_arm_launch.sh           # 终端 1：真机，串口默认 /dev/ttyTHS3
# 另一终端
./nx_arm_demo.sh
```

**手动 launch：**

```bash
# 确认串口设备，并确保有读写权限
sudo usermod -aG dialout $USER   # 首次需要,执行后重新登录

# 无人机 NX（SOP）：/dev/ttyTHS3
ros2 launch manipulator student_arm.launch.py arm_type:=a_l1 port_name:=/dev/ttyTHS3 max_velocity:=0.2 use_rviz:=False

# 个人电脑 / WSL USB：/dev/ttyUSB0（或用 ./pc_arm_launch.sh）
ros2 launch manipulator student_arm.launch.py arm_type:=a_l1 port_name:=/dev/ttyUSB0 max_velocity:=0.2 use_rviz:=False
```

无人机上默认使用 `/dev/ttyTHS3` 串口（如果按照SOP安装将其连接到NX上），如果你是其他串口（比如说连接到自己的电脑上），需要修改 `port_name` 参数或使用电脑版 `pc_*.sh`。

> **机载串口可能被占用：** WindShape 系统服务（`robot.service` / `slave_arm_link_app`）会占用 `/dev/ttyTHS3`。使用本仓库的 `./nx_arm_launch.sh` 等真机入口时，会**自动临时释放**串口供教学节点使用，退出后尽量恢复 `robot.service`。手动 `ros2 launch` 不会自动处理；请用上述快捷脚本，或先 `sudo systemctl stop robot.service`。

### 3.3 launch 参数一览

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_type` | `sim` | `sim` 仿真 / `a_l1` 真机 |
| `port_name` | `/dev/ttyTHS3` | 真机串口(仿真模式忽略)；电脑请改为 `/dev/ttyUSB0` |
| `max_velocity` | `0.5` | 关节最大速度 (rad/s),安全限速 |
| `use_rviz` | `True` | 是否启动 RViz；机载无屏建议 `False` |

## 4. 话题接口

### 4.1 控制指令(你发布)

**话题:** `/student/joint_command`
**类型:** `sensor_msgs/msg/JointState`

| 字段 | 要求 |
|---|---|
| `position` | **必填**,长度必须为 7,单位 rad |
| `velocity` | 选填,前馈速度,单位 rad/s |
| `effort` | 选填,力矩前馈,单位 Nm；**默认控制器会忽略**（见下） |
| `name` | 选填,`joint1` ~ `joint7` |

命令行快速测试:

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### 力矩前馈 (`effort`) 何时生效

默认 `student_arm.launch.py` 使用 `controller_type:=smooth`（`SmoothPositionController`），**只跟踪 `position`，`effort` 写入无效**。

要用力矩前馈，必须把 `student_arm_node` 设为：

| 参数 | 值 | 作用 |
|---|---|---|
| `controller_type` | `mit_stabilization` | MIT 阻抗：`position`/`velocity` + `effort`→力矩前馈 |
| `torque_limit` | 默认 `9.0` | 对 `effort` 再限幅 (Nm) |
| `p_gain` / `d_gain` | j1–j3 **60/1.5**；远端 5/0.1、1/0.1 | 真机默认，见 `stabilization_hw_student_arm.yaml`（2026-07-17 轨迹调参） |

真机末端稳定 launch 已加载 `stabilization_hw_student_arm.yaml`（含上述参数）。自行发力矩时消息示例：

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    effort: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

说明：这是 **MIT 位置阻抗 + 力矩前馈**，不是纯力矩控制；电机侧还会按 `motor_config.yaml` 的 `rated_torque` 限幅。完整用法见 `详细使用手册.md` §3.4 与 `projects/ee-stabilization/README.md`。

### 4.2 状态反馈(你订阅)

| 话题 | 类型 | 内容 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 当前关节位置/速度(RViz 也用它) |
| `/student/joint_feedback` | `dummy_interface/MotorState` | 电机位置/速度/电流/电压/温度(需启动时设参数 `publish_joint_feedback:=true`) |

### 4.3 关节限位

| 关节 | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | joint7 |
|---|---|---|---|---|---|---|---|
| 下限 (rad) | -4.0 | -3.14 | -1.57 | -4.0 | -2.0 | -4.0 | -4.0 |
| 上限 (rad) | 4.0 | 2.0 | 1.57 | 4.0 | 2.0 | 4.0 | 4.0 |

### 4.4 内置安全层(节点自动执行,无需你处理)

1. **限位钳制** — 超限指令被钳到边界并警告
2. **非法指令拒收** — 含 NaN/Inf 或维度 ≠ 7 的指令直接丢弃
3. **超时保持** — 超过 1 s 没收到新指令,机械臂保持当前位置
4. **限速** — 关节速度被限制在 `max_velocity` 以内

> 安全层只是兜底。上真机时仍要从小幅度、低速度开始测试。

## 5. 示例程序

示例位于 `src/arm-platform/demo/`,按由浅入深的顺序学习。运行前确保 `student_arm.launch.py` 已启动。

### 5.1 入门:关节空间摆动 — `move_arm_demo.py`

前 3 个关节做小幅正弦摆动,演示最基本的指令发布。

```bash
cd ~/windylab_ws/src/arm-platform/demo
python3 move_arm_demo.py
```

### 5.2 IK 模块 — `pinocchio_ik.py`

基于 Pinocchio 的逆运动学求解器(阻尼最小二乘迭代),供其他脚本调用,也可单独运行自测:

```bash
python3 pinocchio_ik.py    # 随机 10 点 FK/IK 回代验证
```

在你自己的代码中使用:

```python
import numpy as np
from pinocchio_ik import PinocchioIK

ik = PinocchioIK()
q, ok = ik.solve(np.array([0.4, 0.0, 0.2]))  # 目标末端位置 (base_link 系, m)
pos, rot = ik.forward(q)                      # 正解验证
```

说明:
- 末端 frame 为 `link7`,坐标系原点在 `base_link`
- 默认只解 3D 位置(本臂 joint6/joint7 轴线共线,末端姿态存在病态方向)
- `q_init` 传入上一次的解可热启动,轨迹跟踪时保证关节连续

### 5.3 进阶:末端画圆 — `move_arm_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆,演示笛卡尔空间轨迹 + IK。

```bash
python3 move_arm_ik_demo.py
```

### 5.4 进阶:末端直线往返 — `move_arm_line_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

```bash
python3 move_arm_line_demo.py
```

可修改文件顶部的 `POINT_A` / `POINT_B` / `PERIOD_SEC` 改变运动范围与速度。

## 6. 模型文件

- URDF:`src/arm-platform/config/arm.urdf`(7 自由度,含关节限位,IK 模块直接读取)
- RViz 配置:`src/arm-platform/config/student_arm.rviz`

## 7. 常见问题

**Q: RViz 里模型不动?**
检查是否有节点在发布 `/student/joint_command`,以及指令 `position` 长度是否为 7。

**Q: 终端提示 "Rejected command"?**
指令维度不是 7,或包含 NaN/Inf。

**Q: 提示 "Command timeout, holding current position"?**
正常保护行为:你的程序超过 1 s 没发新指令。连续控制时建议以 ≥ 10 Hz 持续发布。

**Q: IK 不收敛 (ok=False)?**
目标点超出工作空间。本臂大致可达范围:距基座 0.2 ~ 0.6 m。把目标点移近些再试。

**Q: 真机串口打不开?**
确认 `ls /dev/ttyUSB*` 有设备、已加入 dialout 组(需重新登录)、launch 的 `port_name` 正确。

## 8. 真机操作注意事项

1. 先仿真,后真机
2. 上电前确认机械臂周围无人、无障碍物
3. 第一次运行新程序时,把 `max_velocity` 调小(如 0.2)
4. 随时准备急停(断电开关在手边)
5. 程序退出前让机械臂回到安全位姿
