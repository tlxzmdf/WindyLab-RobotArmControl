#!/usr/bin/env bash
# 30s 全仿真录制：机载端固定 + 虚拟主臂轨迹
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
DURATION="${DURATION:-30}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-${PROJECT_DIR}/reports/${STAMP}_teleop_${DURATION}s}"

cd "${ARM_ROOT}/windylab_ws"
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
source install/setup.bash

mkdir -p "${OUT_DIR}"
pkill -f "master_arm_node|ee_stabilization|teleop_stabilization|master_motion_demo|record_teleop_run" 2>/dev/null || true
sleep 0.5

echo "输出目录: ${OUT_DIR}"
echo "录制时长: ${DURATION}s (机载端=static)"
echo ""

ros2 launch "${PROJECT_DIR}/launch/teleop_stabilization.launch.py" \
  master_mode:=scripted use_rviz:=False base_source:=static &
LAUNCH_PID=$!
sleep 3

python3 "${PROJECT_DIR}/scripts/master_motion_demo.py" &
DEMO_PID=$!
sleep 1

cleanup() {
  kill ${DEMO_PID} 2>/dev/null || true
  kill ${LAUNCH_PID} 2>/dev/null || true
  pkill -f "teleop_stabilization.launch|master_motion_demo|record_teleop_run" 2>/dev/null || true
}
trap cleanup EXIT

echo "========== 实时指标 (每 2s) =========="
python3 "${PROJECT_DIR}/scripts/record_teleop_run.py" \
  --duration "${DURATION}" \
  --out "${OUT_DIR}" \
  --live-interval 2.0
RC=$?

echo ""
echo "数据已保存: ${OUT_DIR}"
ls -la "${OUT_DIR}"
exit ${RC}
