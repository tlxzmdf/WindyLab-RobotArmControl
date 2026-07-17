#!/usr/bin/env bash
# 机载 NX · Demo + 数据录制 → arm/run_data/<时间戳>_<demo>/
# 用法: ./nx_arm_record_demo.sh [demo.py] [duration] [sim|a_l1] [sim|real|rviz]
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/nx_arm_env.sh"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"

DEMO="${1:-move_arm_demo.py}"
DURATION="${2:-15}"
ARM_TYPE="${3:-sim}"
LAUNCH_MODE="${4:-sim}"   # sim | real | rviz

DEMO_NAME="${DEMO%.py}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ARM_ROOT}/run_data/${STAMP}_${DEMO_NAME}"
RECORD_PY="${WINDYLAB_WS}/tools/record_demo_run.py"
DEMO_PY="${ARM_DEMO_DIR}/${DEMO}"

if [[ ! -f "${DEMO_PY}" ]]; then
  echo "[FAIL] Demo 不存在: ${DEMO_PY}"
  exit 1
fi

mkdir -p "${ARM_ROOT}/run_data"

CLAIMED=0
if [[ "${LAUNCH_MODE}" == "real" || "${LAUNCH_MODE}" == "rviz" ]]; then
  if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
    echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在"
    exit 1
  fi
  arm_claim_serial "${ARM_SERIAL_PORT}" || exit 1
  CLAIMED=1
fi

case "${LAUNCH_MODE}" in
  sim)
    echo "[INFO] 启动仿真 launch..."
    ros2 launch manipulator student_arm.launch.py arm_type:=sim use_rviz:=False &
    ;;
  real)
    echo "[INFO] 启动真机 launch: ${ARM_SERIAL_PORT}"
    ros2 launch manipulator student_arm.launch.py \
      arm_type:=a_l1 port_name:="${ARM_SERIAL_PORT}" \
      max_velocity:="${ARM_MAX_VELOCITY}" use_rviz:=False &
    ;;
  rviz)
    echo "[INFO] 启动真机+RViz launch..."
    ros2 launch manipulator student_arm.launch.py \
      arm_type:=a_l1 port_name:="${ARM_SERIAL_PORT}" \
      max_velocity:="${ARM_MAX_VELOCITY}" use_rviz:=True &
    ;;
  *)
    echo "[FAIL] 未知 LAUNCH_MODE: ${LAUNCH_MODE} (sim|real|rviz)"
    exit 1
    ;;
esac
LAUNCH_PID=$!

cleanup() {
  kill "${DEMO_PID:-}" 2>/dev/null || true
  kill "${RECORD_PID:-}" 2>/dev/null || true
  kill "${LAUNCH_PID:-}" 2>/dev/null || true
  pkill -f '[s]tudent_arm.launch' 2>/dev/null || true
  pkill -f '[s]tudent_arm_node' 2>/dev/null || true
  if [[ "${CLAIMED}" -eq 1 ]]; then
    arm_release_serial
  fi
}
trap cleanup EXIT

echo "[INFO] 等待 /joint_states..."
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '/joint_states'; then
    break
  fi
  sleep 0.5
done

echo "[INFO] 输出目录: ${OUT_DIR}"
echo "[INFO] 录制 ${DURATION}s，同时运行 ${DEMO} ..."

python3 "${RECORD_PY}" \
  --duration "${DURATION}" \
  --demo "${DEMO}" \
  --arm-type "${ARM_TYPE}" \
  --out "${OUT_DIR}" &
RECORD_PID=$!

sleep 1.0
cd "${ARM_DEMO_DIR}"
python3 "${DEMO}" &
DEMO_PID=$!

sleep "${DURATION}"
kill -INT "${DEMO_PID}" 2>/dev/null || true
wait "${DEMO_PID}" 2>/dev/null || true

wait "${RECORD_PID}" || true

echo ""
echo "========== 录制完成 =========="
echo "目录: ${OUT_DIR}"
if [[ -f "${OUT_DIR}/summary.txt" ]]; then
  cat "${OUT_DIR}/summary.txt"
fi
