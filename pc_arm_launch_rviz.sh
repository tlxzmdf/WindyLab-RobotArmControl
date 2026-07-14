#!/usr/bin/env bash
# 电脑专用版 · 真机 launch + RViz 可视化（终端 1）
# 真机运动 + 屏幕上的仿真模型随 /joint_states 同步显示
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
  echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在。"
  echo "  Windows PowerShell(管理员): usbipd list && usbipd attach --wsl --busid <BUSID>"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]] && [[ ! -e /tmp/.X11-unix/X0 ]] && [[ ! -d /mnt/wslg ]]; then
  echo "[WARN] 未检测到图形环境。RViz 需要 WSLg(Win11) 或 X11。"
  echo "       若窗口无法弹出，可改用: ./pc_arm_launch.sh (无 RViz)"
fi

echo "启动真机 launch + RViz: port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=True
