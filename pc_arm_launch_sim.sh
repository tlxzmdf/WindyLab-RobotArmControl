#!/usr/bin/env bash
# 电脑专用版 · 仿真 launch（终端 1）
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

echo "启动仿真 launch (arm_type:=sim, use_rviz:=False)"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=sim \
  use_rviz:=False
