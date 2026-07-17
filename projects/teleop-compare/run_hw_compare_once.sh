#!/usr/bin/env bash
# 真机对比：手拖一次 → 回放同一轨迹 → 自动录 CLIK + WBC + 出图
# 用法: PORT_NAME=/dev/ttyUSB0 DURATION=15 ./run_hw_compare_once.sh
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
COMPARE_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="${ARM_ROOT}/windylab_ws"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"
PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
DURATION="${DURATION:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION="${COMPARE_DIR}/reports/${STAMP}_compare_once"
TELEOP_CONFIG="${TELEOP_CONFIG:-teleop_compare_mode_b.yaml}"
CLIK_LAUNCH="${ARM_ROOT}/projects/master-slave-stabilization/launch/teleop_stabilization.launch.py"
WBC_LAUNCH="${ARM_ROOT}/projects/master-slave-wbc/launch/teleop_wbc.launch.py"

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "${PORT}" || exit 1
trap 'arm_release_serial' EXIT INT TERM

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"
colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description arm_ee_stabilization_control arm_teleop_wbc_control manipulator
source install/setup.bash

MANIP_SHARE="$(ros2 pkg prefix manipulator)/share/manipulator"
MASTER_URDF="${ARM_ROOT}/projects/master-slave-wbc/urdf/arm_link7_zero_mass.urdf"

kill_all() {
  pkill -f "master_arm_node|ee_stabilization|teleop_stabilization|teleop_wbc|teleop_wbc.launch|record_master_drag|play_master_joints|record_compare" 2>/dev/null || true
  sleep 0.5
}

mkdir -p "${SESSION}"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  一次手拖 · 双方案对比（CLIK vs WBC）                         ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  阶段 1: 真机手拖 ${DURATION}s → 保存主臂轨迹                  ║"
echo "║  阶段 2: 回放轨迹 + CLIK 从臂 → 录制                          ║"
echo "║  阶段 3: 回放轨迹 + WBC  从臂 → 录制                          ║"
echo "║  阶段 4: 对比曲线 + 急动/急停分析                             ║"
echo "║  port=${PORT}  无 RViz                                       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ── 阶段 1：真机手拖，只录主臂 ──
kill_all
echo ">>> 阶段 1/4: 请手拖主臂 ${DURATION}s ..."
ros2 run manipulator master_arm_node --ros-args \
  -r __ns:=/master \
  -p arm_type:=a_l1 \
  -p arm_version:=gamma \
  -p auto_reset:=false \
  -p port_name:="${PORT}" \
  -p motor_config_path:="${MANIP_SHARE}/motor_config.yaml" \
  -p arm_config_path:="${MANIP_SHARE}/arm_config.yaml" \
  -p MAX_TORQUE:=3.0 \
  -p GRAVITY:=9.81 \
  -p publish_joint_state:=true \
  -p publish_joint_feedback:=false \
  -p urdf_path:="${MASTER_URDF}" &
MASTER_PID=$!
sleep 3

python3 "${COMPARE_DIR}/scripts/record_master_drag.py" \
  --duration "${DURATION}" \
  --out "${SESSION}/master"

kill "${MASTER_PID}" 2>/dev/null || true
kill_all

MASTER_CSV="${SESSION}/master/master_joints.csv"
if [ ! -s "${MASTER_CSV}" ]; then
  echo "错误: 主臂轨迹未录到，请检查串口 ${PORT}" >&2
  exit 1
fi

# 读取实际时长（可能比 DURATION 略短）
PLAY_DURATION="${DURATION}"
if [ -f "${SESSION}/master/master_meta.json" ]; then
  PLAY_DURATION="$(python3 -c "import json; print(json.load(open('${SESSION}/master/master_meta.json'))['duration_sec']+1.0)")"
fi

run_replay_method() {
  local method="$1"
  local launch="$2"
  local out="${SESSION}/${method}"
  local pkill_pat="$3"
  mkdir -p "${out}"

  echo ""
  echo ">>> 回放主臂轨迹 + ${method^^} 从臂控制 ..."
  kill_all

  ros2 launch "${launch}" \
    master_mode:=scripted \
    teleop_config:="${TELEOP_CONFIG}" \
    use_rviz:=False &
  local lp=$!
  sleep 3

  python3 "${COMPARE_DIR}/scripts/play_master_joints.py" "${MASTER_CSV}" &
  local pp=$!
  sleep 1

  python3 "${COMPARE_DIR}/scripts/record_compare_run.py" \
    --method "${method}" \
    --duration "${PLAY_DURATION}" \
    --out "${out}" \
    --live-interval 3.0

  cp "${MASTER_CSV}" "${out}/master_joints.csv" 2>/dev/null || true
  cp "${SESSION}/master/master_meta.json" "${out}/master_meta.json" 2>/dev/null || true

  kill "${pp}" "${lp}" 2>/dev/null || true
  pkill -f "${pkill_pat}|play_master_joints" 2>/dev/null || true
  sleep 0.5
}

# ── 阶段 2 & 3：同一轨迹，分别跑 CLIK / WBC ──
run_replay_method clik "${CLIK_LAUNCH}" "ee_stabilization|teleop_stabilization"
run_replay_method wbc  "${WBC_LAUNCH}"  "teleop_wbc|teleop_wbc.launch"

# ── 阶段 4：出图 + 急动分析 ──
echo ""
echo ">>> 阶段 4/4: 生成对比曲线与急动/急停分析 ..."
python3 "${COMPARE_DIR}/scripts/benchmark_solvers.py" \
  --out "${SESSION}/solver_benchmark.json" || true
python3 "${COMPARE_DIR}/scripts/plot_compare.py" \
  --clik "${SESSION}/clik" \
  --wbc  "${SESSION}/wbc" \
  --out  "${SESSION}/plots"
python3 "${COMPARE_DIR}/scripts/analyze_abrupt_motion.py" \
  --session "${SESSION}" \
  --out "${SESSION}/abrupt_motion" || true

kill_all

echo ""
echo "完成。会话目录: ${SESSION}"
echo "  主臂轨迹: ${MASTER_CSV}"
echo "  CLIK 数据: ${SESSION}/clik/"
echo "  WBC  数据: ${SESSION}/wbc/"
echo "  对比曲线: ${SESSION}/plots/compare_timeseries.png"
echo "  对比曲线: ${SESSION}/plots/compare_bars.png"
echo "  急动分析: ${SESSION}/abrupt_motion/ABRUPT_MOTION_REPORT.md"
echo "  急动曲线: ${SESSION}/abrupt_motion/abrupt_motion_analysis.png"
if [ -f "${SESSION}/plots/COMPARE_RESULT.md" ]; then
  cat "${SESSION}/plots/COMPARE_RESULT.md"
fi
if [ -f "${SESSION}/abrupt_motion/ABRUPT_MOTION_REPORT.md" ]; then
  echo ""
  cat "${SESSION}/abrupt_motion/ABRUPT_MOTION_REPORT.md"
fi
