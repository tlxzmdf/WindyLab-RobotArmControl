#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

MODE="${1:-A}"
ARM_TYPE="${ARM_TYPE:-a_l1}"
PORT="${PORT_NAME:-/dev/ttyTHS3}"
BASE_SOURCE="${BASE_SOURCE:-simulated}"

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control manipulator
source install/setup.bash

pkill -f "ee_stabilization|stabilization.launch|student_arm_node" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  机头稳定 — 真机模式 ${MODE}                               ║
╠══════════════════════════════════════════════════════════╣
║  A: IK + MIT 位置跟踪                                    ║
║  B: IK + 计算力矩前馈 (MIT current)                      ║
║  C: 任务空间 OSC 力矩前馈 (MIT current)                  ║
║                                                          ║
║  arm_type=${ARM_TYPE}  port=${PORT}                      ║
║  base_source=${BASE_SOURCE}                              ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch arm_ee_stabilization_description stabilization_hardware.launch.py \
  stabilization_mode:="${MODE}" \
  arm_type:="${ARM_TYPE}" \
  port_name:="${PORT}" \
  base_source:="${BASE_SOURCE}" \
  use_rviz:=False
