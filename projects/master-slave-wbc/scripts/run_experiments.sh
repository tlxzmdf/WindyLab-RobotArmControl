#!/usr/bin/env bash
# 批量仿真实验：Mode A / Mode B，各 30s 数据采集
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ARM_ROOT}/windylab_ws"
REPORT_ROOT="${PROJECT_DIR}/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
DURATION="${DURATION:-30}"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"
colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_teleop_wbc_control manipulator
source install/setup.bash

kill_all() {
  pkill -f "teleop_wbc|teleop_wbc.launch|master_motion_demo|record_wbc_run|rviz2" 2>/dev/null || true
  sleep 0.5
}

run_case() {
  local label="$1"
  local config="$2"
  local out="${REPORT_ROOT}/${STAMP}_${label}"
  mkdir -p "${out}"

  echo ""
  echo "========== ${label} (${config}) =========="
  kill_all

  ros2 launch "${PROJECT_DIR}/launch/teleop_wbc.launch.py" \
    master_mode:=scripted \
    teleop_config:="${config}" \
    use_rviz:=False &
  local launch_pid=$!
  sleep 3

  python3 "${PROJECT_DIR}/scripts/master_motion_demo.py" &
  local demo_pid=$!
  sleep 1

  python3 "${PROJECT_DIR}/scripts/record_wbc_run.py" \
    --duration "${DURATION}" \
    --out "${out}" \
    --label "${label}" \
    --live-interval 5.0
  local rc=$?

  kill "${demo_pid}" 2>/dev/null || true
  kill "${launch_pid}" 2>/dev/null || true
  kill_all

  echo "结果目录: ${out}"
  return "${rc}"
}

mkdir -p "${REPORT_ROOT}"
echo "实验批次: ${STAMP}  duration=${DURATION}s"

run_case "mode_a_mirror" "teleop_wbc.yaml" || true
run_case "mode_b_wbc" "teleop_wbc_disturbed.yaml" || true

python3 "${PROJECT_DIR}/scripts/analyze_wbc_runs.py" \
  --batch "${REPORT_ROOT}/${STAMP}_*" \
  --out "${REPORT_ROOT}/${STAMP}_comparison.md"

echo ""
echo "完成。对比报告: ${REPORT_ROOT}/${STAMP}_comparison.md"
