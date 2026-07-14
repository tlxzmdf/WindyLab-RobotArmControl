#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select arm_ee_stabilization_description arm_ee_stabilization_control
source install/setup.bash

# 清理旧仿真进程，避免多个 robot_state_publisher 抢 TF
pkill -f "arm_ee_stabilization|stabilization.launch|tracking_control|arm-lagrangian-tracking" 2>/dev/null || true
pkill -f "robot_state_publisher.*launch_params" 2>/dev/null || true
sleep 0.5

cat <<'EOF'

╔══════════════════════════════════════════════════════════╗
║  机械臂末端位姿稳定 — RViz 仿真                          ║
╠══════════════════════════════════════════════════════════╣
║  场景: 机载端 (base_link) 在世界原点球内大幅度运动         ║
║  目标: 末端 (link6) 在世界系中位置+方向保持固定           ║
║                                                          ║
║  机械臂两端:                                             ║
║    机载端·动  — 随无人机连接处大幅摆动 (橙色球+轨迹)     ║
║    末端·固定  — 世界系锁定 (绿色球+坐标轴不动)           ║
║                                                          ║
║  RViz 显示:                                              ║
║    黄色半透明球 = 扰动范围 (圆心: 世界原点)              ║
║    橙色大球+轨迹 = 机载端 (大幅运动)                     ║
║    绿色球+坐标轴 = 末端锁定目标 (固定不动)               ║
║    橙色小球     = 实际末端 (应贴合绿色球)                ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch arm_ee_stabilization_description stabilization.launch.py
