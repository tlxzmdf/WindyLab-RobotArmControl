#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 保留命令行传入的限速（source .pc_arm_env.sh 会覆盖为 0.2）
CLI_MAX_VELOCITY="${ARM_MAX_VELOCITY:-}"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"
if [[ -n "${CLI_MAX_VELOCITY}" ]]; then
  export ARM_MAX_VELOCITY="${CLI_MAX_VELOCITY}"
fi

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
  echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在。"
  echo "  Windows PowerShell(管理员): usbipd list && usbipd attach --wsl --busid <BUSID>"
  exit 1
fi

echo "启动真机 launch: port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=False
