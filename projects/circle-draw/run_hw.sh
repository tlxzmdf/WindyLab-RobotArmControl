#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh" 2>/dev/null || {
  source /opt/ros/humble/setup.bash
  source "${ARM_ROOT}/windylab_ws/install/setup.bash"
}

MODE="${MODE:-diff}"
MAX_VEL="${ARM_MAX_VELOCITY:-0.35}"
PORT="${ARM_SERIAL_PORT:-/dev/ttyUSB0}"

pkill -f "student_arm_node|circle_draw_node|move_arm_ik_demo" 2>/dev/null || true
sleep 0.5

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  末端画圆 — 真机 (circle-draw)                           ║
╠══════════════════════════════════════════════════════════╣
║  模式: ${MODE}   port: ${PORT}                           ║
║  max_velocity: ${MAX_VEL} rad/s (可用 ARM_MAX_VELOCITY)  ║
╚══════════════════════════════════════════════════════════╝

EOF

ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${PORT}" \
  max_velocity:="${MAX_VEL}" \
  use_rviz:=False &
LAUNCH_PID=$!
sleep 2

trap 'kill ${LAUNCH_PID} 2>/dev/null || true' EXIT
cd "${PROJECT_DIR}/scripts"
exec python3 circle_draw_node.py --mode "${MODE}" --max-joint-velocity "${MAX_VEL}"
