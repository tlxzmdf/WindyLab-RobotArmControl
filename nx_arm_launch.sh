#!/usr/bin/env bash
# 机载 NX · 真机 launch（终端 1，无 RViz）
# 串口默认 /dev/ttyTHS3（SOP 接 NX UART）
# 若串口被 WindShape robot.service 等占用，会临时释放，退出后尝试恢复。
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
  echo "  机载 SOP 默认为 /dev/ttyTHS3。请检查接线与设备节点:"
  echo "    ls -la /dev/ttyTHS*"
  exit 1
fi

arm_claim_serial "${ARM_SERIAL_PORT}" || exit 1
trap 'arm_release_serial' EXIT INT TERM

echo "启动真机 launch (NX): port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=False
