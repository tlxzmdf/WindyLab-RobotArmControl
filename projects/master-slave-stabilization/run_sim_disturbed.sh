#!/usr/bin/env bash
# 模式 B 仿真：机载端随机扰动 + 虚拟主臂
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control manipulator
source install/setup.bash

pkill -f "master_arm_node|ee_stabilization|teleop_stabilization|master_motion_demo|mount_disturbance_profile" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  模式 B 仿真：机载端随机扰动 + 虚拟主臂                   ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch "${PROJECT_DIR}/launch/teleop_stabilization.launch.py" \
  master_mode:=scripted \
  teleop_config:=teleop_stabilization_disturbed.yaml \
  use_rviz:=True &
LAUNCH_PID=$!
sleep 3

trap 'kill ${LAUNCH_PID} 2>/dev/null || true' EXIT
cd "${PROJECT_DIR}/scripts"
exec python3 master_motion_demo.py
