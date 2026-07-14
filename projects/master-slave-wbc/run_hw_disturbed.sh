#!/usr/bin/env bash
# 模式 B：真机主臂 + 仿真从臂，机载端随机扰动，QP-WBC + 积分动作
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

PORT="${PORT_NAME:-/dev/ttyUSB0}"
USE_RVIZ="${USE_RVIZ:-True}"
TELEOP_CONFIG="${TELEOP_CONFIG:-teleop_wbc_disturbed.yaml}"

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_teleop_wbc_control manipulator
source install/setup.bash

pkill -f "master_arm_node|student_arm_node|teleop_wbc|teleop_wbc.launch|master_motion_demo|mount_disturbance_profile" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  模式 B：机载端扰动 + QP-WBC + 积分动作                   ║
╠══════════════════════════════════════════════════════════╣
║  主臂: 真机零力手拖                                       ║
║  从臂: 末端世界系目标 + 速度层 QP-WBC                     ║
║  port=${PORT}                                             ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch "${PROJECT_DIR}/launch/teleop_wbc.launch.py" \
  master_arm_type:=a_l1 \
  master_mode:=backdrive \
  port_name:="${PORT}" \
  teleop_config:="${TELEOP_CONFIG}" \
  use_rviz:="${USE_RVIZ}"
