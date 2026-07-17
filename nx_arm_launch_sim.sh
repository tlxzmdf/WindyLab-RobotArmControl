#!/usr/bin/env bash
# 机载 NX · 仿真 launch（终端 1）
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/nx_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

echo "启动仿真 launch (NX, arm_type:=sim, use_rviz:=False)"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=sim \
  use_rviz:=False
