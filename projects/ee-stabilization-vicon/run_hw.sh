#!/usr/bin/env bash
# ee-stabilization-vicon · 真机：Vicon 飞机相对 t0 扰动 + 末端世界系稳定
# 不修改 projects/ee-stabilization；复用 windylab_ws 中已有 ROS 包。
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARM_ROOT="$(cd "${PROJECT_DIR}/../.." && pwd)"
cd "${ARM_ROOT}/windylab_ws"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"

MODE="${1:-A}"
ARM_TYPE="${ARM_TYPE:-a_l1}"
PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
# 本项目默认 tf（由 vicon_relative_bridge 发相对 Δ）；需要对照原球扰动时可 BASE_SOURCE=simulated
BASE_SOURCE="${BASE_SOURCE:-tf}"
POSE_TOPIC="${POSE_TOPIC:-/vrpn/pregme/pose}"
LATCH_DELAY="${LATCH_DELAY:-2.0}"
MAX_VEL="${ARM_MAX_VELOCITY:-0.25}"
# claim_arm_serial 会停 robot.service / robot.launch，机上随其启动的 VRPN 会被带走。
# 默认由本 launch 再拉起 ~/zihan_ws/vicon_perception；仅当你已在 claim 之后自启 VRPN 时设 START_VRPN=false。
START_VRPN="${START_VRPN:-true}"
VICON_WS="${VICON_WS:-${HOME}/zihan_ws/vicon_perception/src}"
USE_RVIZ="${USE_RVIZ:-False}"
# 测试前先回 q_home（默认开）；SKIP_HOME=1 可跳过
HOME_BEFORE="${HOME_BEFORE_STABILIZE:-true}"
if [[ "${SKIP_HOME:-0}" == "1" || "${SKIP_HOME:-false}" == "true" ]]; then
  HOME_BEFORE="false"
fi
HOME_DURATION="${HOME_DURATION:-6.0}"
HOME_SETTLE="${HOME_SETTLE:-0.8}"

if [[ "${BASE_SOURCE}" == "tf" && "${START_VRPN}" =~ ^(true|1|yes|True|TRUE)$ ]]; then
  if [[ ! -f "${VICON_WS}/install/setup.bash" ]]; then
    echo "[FAIL] START_VRPN=true 但找不到 ${VICON_WS}/install/setup.bash"
    echo "       请编译 vicon_perception，或设置 VICON_WS=/path/to/vicon_perception/src"
    exit 1
  fi
fi

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "${PORT}" || exit 1
trap 'arm_release_serial' EXIT INT TERM

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control manipulator
source install/setup.bash

# 不杀 vrpn_listener：claim 可能已停掉 robot.service 自带的 VRPN；本 launch 会按 start_vrpn 再起一份
pkill -f "ee_stabilization|stabilization.launch|student_arm_node|vicon_relative_bridge|move_to_home" 2>/dev/null || true
sleep 0.5

if [[ "${BASE_SOURCE}" == "tf" && ! "${START_VRPN}" =~ ^(true|1|yes|True|TRUE)$ ]]; then
  echo "[WARN] START_VRPN=false：请确认 claim 串口之后 ${POSE_TOPIC} 仍在发布，"
  echo "       否则 bridge 无法 latch，控制端 mount 恒为 0（晃机无反应）。"
fi

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  机头稳定 · Vicon 相对扰动 — 真机模式 ${MODE}              ║
╠══════════════════════════════════════════════════════════╣
║  飞机 pose: ${POSE_TOPIC}                                ║
║  扰动定义:  Δ = inv(T_plane(t0)) · T_plane(t)              ║
║  末端目标:  启动锁定后的世界系位姿（抵消飞机相对运动）     ║
║  回 home: ${HOME_BEFORE}  (${HOME_DURATION}s + settle ${HOME_SETTLE}s)
║  base_source=${BASE_SOURCE}  port=${PORT}                ║
║  start_vrpn=${START_VRPN}  vicon_ws=${VICON_WS}
║  latch_delay=${LATCH_DELAY}s  max_vel=${MAX_VEL}         ║
║  重新冻结 t0:  ros2 service call /vicon_relative_bridge/latch_t0 std_srvs/srv/Trigger {}
╚══════════════════════════════════════════════════════════╝

EOF

# Optional last-merge YAML for ee_stabilization (Mode C auto-tune).
PARAMS_OVERLAY="${PARAMS_OVERLAY:-}"

if [[ "${BASE_SOURCE}" == "tf" ]]; then
  LAUNCH_ARGS=(
    stabilization_mode:="${MODE}"
    arm_type:="${ARM_TYPE}"
    port_name:="${PORT}"
    max_velocity:="${MAX_VEL}"
    pose_topic:="${POSE_TOPIC}"
    latch_delay:="${LATCH_DELAY}"
    start_vrpn:="${START_VRPN}"
    vicon_ws:="${VICON_WS}"
    use_rviz:="${USE_RVIZ}"
    home_before_stabilize:="${HOME_BEFORE}"
    home_duration:="${HOME_DURATION}"
    home_settle:="${HOME_SETTLE}"
  )
  if [[ -n "${PARAMS_OVERLAY}" ]]; then
    LAUNCH_ARGS+=(params_overlay:="${PARAMS_OVERLAY}")
    echo "[INFO] params_overlay=${PARAMS_OVERLAY}"
  fi
  exec ros2 launch "${PROJECT_DIR}/launch/stabilization_vicon_hw.launch.py" "${LAUNCH_ARGS[@]}"
else
  # 回退：不启 bridge，与原版行为一致（例如 simulated）
  exec ros2 launch arm_ee_stabilization_description stabilization_hardware.launch.py \
    stabilization_mode:="${MODE}" \
    arm_type:="${ARM_TYPE}" \
    port_name:="${PORT}" \
    max_velocity:="${MAX_VEL}" \
    base_source:="${BASE_SOURCE}" \
    use_rviz:="${USE_RVIZ}"
fi
