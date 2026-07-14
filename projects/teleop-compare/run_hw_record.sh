#!/usr/bin/env bash
# 真机 Mode B 对比：无 RViz，手拖 15s 录制
# 用法: METHOD=clik|wbc PORT_NAME=/dev/ttyUSB0 ./run_hw_record.sh
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
COMPARE_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="${ARM_ROOT}/windylab_ws"
METHOD="${METHOD:?请设置 METHOD=clik 或 METHOD=wbc}"
PORT="${PORT_NAME:-/dev/ttyUSB0}"
DURATION="${DURATION:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${COMPARE_DIR}/reports/${STAMP}_${METHOD}"
TELEOP_CONFIG="${TELEOP_CONFIG:-teleop_compare_mode_b.yaml}"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
cd "${WS}"

if [ "${METHOD}" = "clik" ]; then
  PKG="arm_ee_stabilization_control"
  PROJECT="${ARM_ROOT}/projects/master-slave-stabilization"
  LAUNCH="${PROJECT}/launch/teleop_stabilization.launch.py"
  PKILL='ee_stabilization|teleop_stabilization'
else
  PKG="arm_teleop_wbc_control"
  PROJECT="${ARM_ROOT}/projects/master-slave-wbc"
  LAUNCH="${PROJECT}/launch/teleop_wbc.launch.py"
  PKILL='teleop_wbc|teleop_wbc.launch'
fi

colcon build --symlink-install \
  --packages-select arm_ee_stabilization_description "${PKG}" manipulator
source install/setup.bash

pkill -f "master_arm_node|${PKILL}|record_compare" 2>/dev/null || true
sleep 0.5
mkdir -p "${OUT}"

echo "=== METHOD=${METHOD}  DURATION=${DURATION}s  OUT=${OUT} ==="
echo "=== 配置: ${TELEOP_CONFIG}  无 RViz ==="

ros2 launch "${LAUNCH}" \
  master_arm_type:=a_l1 \
  master_mode:=backdrive \
  port_name:="${PORT}" \
  teleop_config:="${TELEOP_CONFIG}" \
  use_rviz:=False &
LP=$!
sleep 4

python3 "${COMPARE_DIR}/scripts/record_compare_run.py" \
  --method "${METHOD}" \
  --duration "${DURATION}" \
  --out "${OUT}" \
  --live-interval 3.0

kill "${LP}" 2>/dev/null || true
pkill -f "master_arm_node|${PKILL}" 2>/dev/null || true

echo "完成: ${OUT}"
echo "下一步: python3 ${COMPARE_DIR}/scripts/plot_compare.py --clik ... --wbc ..."
