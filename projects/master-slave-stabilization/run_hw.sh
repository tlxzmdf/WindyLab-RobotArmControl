#!/usr/bin/env bash
# 模式 A：真机主臂 + 仿真从臂，机载端固定，关节直接映射
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"
PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
USE_RVIZ="${USE_RVIZ:-True}"

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "${PORT}" || exit 1
trap 'arm_release_serial' EXIT INT TERM

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control manipulator
source install/setup.bash

pkill -f "master_arm_node|student_arm_node|ee_stabilization|teleop_stabilization|master_command_demo|master_motion_demo|mount_disturbance_profile" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  模式 A：机载端固定                                       ║
╠══════════════════════════════════════════════════════════╣
║  主臂: 真机零力手拖                                       ║
║  从臂: 直接复制主臂关节角 (joint_mirror)，无 IK 跳变      ║
║  机载端: 固定不动                                         ║
║  port=${PORT}                                             ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch "${PROJECT_DIR}/launch/teleop_stabilization.launch.py" \
  master_arm_type:=a_l1 \
  master_mode:=backdrive \
  port_name:="${PORT}" \
  teleop_config:=teleop_stabilization.yaml \
  use_rviz:="${USE_RVIZ}"
