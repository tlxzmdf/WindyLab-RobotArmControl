#!/usr/bin/env bash
# 仿真对比（无 RViz）：依次跑 CLIK / WBC 各 15s，生成曲线
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
COMPARE_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="${ARM_ROOT}/windylab_ws"
DURATION="${DURATION:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="${COMPARE_DIR}/reports/${STAMP}_sim_compare"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  colcon build --symlink-install \
    --packages-select arm_ee_stabilization_description arm_ee_stabilization_control arm_teleop_wbc_control manipulator
fi
source install/setup.bash

run_one() {
  local method="$1"
  local out="${REPORT}/${method}"
  mkdir -p "${out}"
  pkill -f "master_motion_demo|ee_stabilization|teleop_wbc|record_compare" 2>/dev/null || true
  sleep 0.5

  if [ "${method}" = "clik" ]; then
    ros2 launch "${ARM_ROOT}/projects/master-slave-stabilization/launch/teleop_stabilization.launch.py" \
      master_mode:=scripted teleop_config:=teleop_compare_mode_b.yaml use_rviz:=False &
  else
    ros2 launch "${ARM_ROOT}/projects/master-slave-wbc/launch/teleop_wbc.launch.py" \
      master_mode:=scripted teleop_config:=teleop_compare_mode_b.yaml use_rviz:=False &
  fi
  local lp=$!
  sleep 3
  python3 "${ARM_ROOT}/projects/master-slave-wbc/scripts/master_motion_demo.py" &
  local dp=$!
  sleep 1
  python3 "${COMPARE_DIR}/scripts/record_compare_run.py" \
    --method "${method}" --duration "${DURATION}" --out "${out}"
  kill $dp $lp 2>/dev/null || true
  pkill -f "master_motion_demo|ee_stabilization|teleop_wbc" 2>/dev/null || true
}

mkdir -p "${REPORT}"
run_one clik
run_one wbc

python3 "${COMPARE_DIR}/scripts/benchmark_solvers.py" \
  --out "${REPORT}/solver_benchmark.json"

python3 "${COMPARE_DIR}/scripts/plot_compare.py" \
  --clik "${REPORT}/clik" \
  --wbc "${REPORT}/wbc" \
  --out "${REPORT}/compare_plots"

python3 "${COMPARE_DIR}/scripts/analyze_abrupt_motion.py" \
  --session "${REPORT}" \
  --out "${REPORT}/abrupt_motion" || true

echo "报告目录: ${REPORT}"
ls -la "${REPORT}/compare_plots" 2>/dev/null || true
ls -la "${REPORT}/abrupt_motion" 2>/dev/null || true
