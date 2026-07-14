#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

MODE="${MODE:-diff}"
MAX_VEL="${MAX_VEL:-0.5}"

colcon build --symlink-install --packages-select manipulator 2>/dev/null || true
source install/setup.bash

pkill -f "student_arm_node|circle_draw_node|move_arm_ik_demo" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  末端画圆 — 优化版 (circle-draw)                         ║
╠══════════════════════════════════════════════════════════╣
║  模式: ${MODE}  (diff | precompute)                      ║
║  max_velocity: ${MAX_VEL} rad/s                          ║
║                                                          ║
║  相对 move_arm_ik_demo.py:                               ║
║    · 100 Hz 指令 + 关节速度前馈                          ║
║    · 微分 IK / 离线轨迹，避免限速控制器滞后              ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch manipulator student_arm.launch.py \
  arm_type:=sim \
  max_velocity:="${MAX_VEL}" \
  kinematic_mode:=False \
  use_rviz:=True &
LAUNCH_PID=$!
sleep 2

trap 'kill ${LAUNCH_PID} 2>/dev/null || true' EXIT
cd "${PROJECT_DIR}/scripts"
exec python3 circle_draw_node.py --mode "${MODE}" --max-joint-velocity "${MAX_VEL}"
