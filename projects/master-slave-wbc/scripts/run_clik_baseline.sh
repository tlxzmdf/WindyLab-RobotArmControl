#!/usr/bin/env bash
# 与 master-slave-stabilization (CLIK) 对比的数据采集
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
WBC_PROJECT="$(cd "$(dirname "$0")/.." && pwd)"
STAB_PROJECT="${ARM_ROOT}/projects/master-slave-stabilization"
WS="${ARM_ROOT}/windylab_ws"
DURATION="${DURATION:-25}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${WBC_PROJECT}/reports/${STAMP}_mode_b_clik_baseline"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"
colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control manipulator
source install/setup.bash

pkill -f "ee_stabilization|teleop_stabilization|master_motion_demo|record_" 2>/dev/null || true
sleep 0.5

mkdir -p "${OUT}"
ros2 launch "${STAB_PROJECT}/launch/teleop_stabilization.launch.py" \
  master_mode:=scripted \
  teleop_config:=teleop_stabilization_disturbed.yaml \
  use_rviz:=False &
LP=$!
sleep 3
python3 "${WBC_PROJECT}/scripts/master_motion_demo.py" &
DP=$!
sleep 1

python3 "${STAB_PROJECT}/scripts/record_teleop_run.py" \
  --duration "${DURATION}" \
  --out "${OUT}" \
  --live-interval 5.0 || true

kill $DP $LP 2>/dev/null || true
pkill -f "ee_stabilization|master_motion_demo" 2>/dev/null || true

echo "CLIK baseline: ${OUT}"
if [ -f "${OUT}/summary.txt" ]; then cat "${OUT}/summary.txt"; fi
