#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Prefer NX env on Jetson; keep PC env for WSL/laptop (互不影响)
# shellcheck disable=SC1091
if [[ -f /etc/nv_tegra_release || -e /sys/module/tegra_fuse || "${ARM_PLATFORM:-}" == "nx" ]] \
  && [[ -f "${ARM_ROOT}/.nx_arm_env.sh" ]]; then
  source "${ARM_ROOT}/.nx_arm_env.sh"
elif [[ -f "${ARM_ROOT}/.pc_arm_env.sh" ]]; then
  source "${ARM_ROOT}/.pc_arm_env.sh"
else
  source /opt/ros/humble/setup.bash
  source "${ARM_ROOT}/windylab_ws/install/setup.bash"
fi

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"

MODE="${MODE:-diff}"
MAX_VEL="${ARM_MAX_VELOCITY:-0.35}"
PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "${PORT}" || exit 1

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

cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  pkill -f 'circle_draw_node' 2>/dev/null || true
  arm_release_serial
}
trap cleanup EXIT
cd "${PROJECT_DIR}/scripts"
python3 circle_draw_node.py --mode "${MODE}" --max-joint-velocity "${MAX_VEL}"
