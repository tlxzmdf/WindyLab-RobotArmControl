#!/usr/bin/env bash
# 机载 NX · 真机 launch + RViz（终端 1）
# 需本机图形环境（DISPLAY / Weston）；无屏请用 ./nx_arm_launch.sh
# 若串口被占用，会临时释放，退出后尝试恢复。
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLI_MAX_VELOCITY="${ARM_MAX_VELOCITY:-}"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/nx_arm_env.sh"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
if [[ -n "${CLI_MAX_VELOCITY}" ]]; then
  export ARM_MAX_VELOCITY="${CLI_MAX_VELOCITY}"
fi

if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
  echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在。"
  echo "  机载 SOP 默认为 /dev/ttyTHS3。请检查: ls -la /dev/ttyTHS*"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]] && [[ ! -e /tmp/.X11-unix/X0 ]]; then
  echo "[WARN] 未检测到图形环境。无人机上常见无屏运行，可改用: ./nx_arm_launch.sh"
fi

arm_claim_serial "${ARM_SERIAL_PORT}" || exit 1
trap 'arm_release_serial' EXIT INT TERM

echo "启动真机 launch + RViz (NX): port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=True
