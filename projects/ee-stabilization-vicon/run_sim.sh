#!/usr/bin/env bash
# ee-stabilization-vicon · 仿真（Vicon 相对扰动 → external）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARM_ROOT="$(cd "${PROJECT_DIR}/../.." && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

POSE_TOPIC="${POSE_TOPIC:-/vrpn/pregme/pose}"
LATCH_DELAY="${LATCH_DELAY:-2.0}"
START_VRPN="${START_VRPN:-false}"
VICON_WS="${VICON_WS:-${HOME}/zihan_ws/vicon_perception/src}"
USE_RVIZ="${USE_RVIZ:-True}"

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control
source install/setup.bash

pkill -f "arm_ee_stabilization|stabilization.launch|vicon_relative_bridge" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  机头稳定 · Vicon 相对扰动 — 仿真                          ║
║  Δ = inv(T_plane(t0)) · T_plane(t) → /mount_disturbance  ║
║  pose: ${POSE_TOPIC}                                     ║
╚══════════════════════════════════════════════════════════╝

EOF

exec ros2 launch "${PROJECT_DIR}/launch/stabilization_vicon_sim.launch.py" \
  pose_topic:="${POSE_TOPIC}" \
  latch_delay:="${LATCH_DELAY}" \
  start_vrpn:="${START_VRPN}" \
  vicon_ws:="${VICON_WS}" \
  use_rviz:="${USE_RVIZ}"
