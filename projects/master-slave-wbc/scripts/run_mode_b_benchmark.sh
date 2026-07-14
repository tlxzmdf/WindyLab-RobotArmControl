#!/usr/bin/env bash
# 仅 Mode B 25s 数据采集（改进后快速验证）
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ARM_ROOT}/windylab_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
DURATION="${DURATION:-25}"
OUT="${PROJECT_DIR}/reports/${STAMP}_mode_b_wbc"
LABEL="${1:-mode_b_wbc}"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"
colcon build --symlink-install --packages-select arm_teleop_wbc_control arm_ee_stabilization_description manipulator
source install/setup.bash

pkill -f "teleop_wbc|master_motion_demo|record_wbc" 2>/dev/null || true
sleep 0.5
mkdir -p "${OUT}"

ros2 launch "${PROJECT_DIR}/launch/teleop_wbc.launch.py" \
  master_mode:=scripted teleop_config:=teleop_wbc_disturbed.yaml use_rviz:=False &
LP=$!
sleep 3
python3 "${PROJECT_DIR}/scripts/master_motion_demo.py" &
DP=$!
sleep 1

python3 "${PROJECT_DIR}/scripts/record_wbc_run.py" \
  --duration "${DURATION}" --out "${OUT}" --label "${LABEL}" --live-interval 5.0

kill $DP $LP 2>/dev/null || true
pkill -f teleop_wbc 2>/dev/null || true
echo "OUT=${OUT}"
