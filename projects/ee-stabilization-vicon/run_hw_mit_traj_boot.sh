#!/usr/bin/env bash
# Start MIT student_arm + cosine/line/circle HW traj test.
# Kill competitors by PID only (never pkill -f self-matching patterns).
set -eo pipefail

PROJECT="$(cd "$(dirname "$0")" && pwd)"
ARM_ROOT="$(cd "${PROJECT}/../.." && pwd)"
WS="${ARM_ROOT}/windylab_ws"
LOGDIR="${PROJECT}/data/mit_traj"
mkdir -p "${LOGDIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
STUDENT_LOG="${LOGDIR}/student_arm_${STAMP}.log"
TEST_LOG="${LOGDIR}/test_run_${STAMP}.log"

set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"

PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
echo "[boot] port=${PORT}" | tee -a "${TEST_LOG}"

kill_matching_pids() {
  local needle="$1"
  local self=$$
  local pid cmd
  while read -r pid cmd; do
    [[ -z "${pid}" ]] && continue
    [[ "${pid}" == "${self}" ]] && continue
    [[ "${pid}" == "${PPID}" ]] && continue
    # skip this script's process tree roots that are bash wrappers running us
    case "${cmd}" in
      *run_hw_mit_traj_boot*) continue ;;
    esac
    echo "[boot] kill pid=${pid} cmd=${cmd}" | tee -a "${TEST_LOG}"
    kill -TERM "${pid}" 2>/dev/null || true
  done < <(ps -eo pid=,args= | grep -F "${needle}" | grep -v grep || true)
}

# Competitors (literal needles must NOT appear as substrings of this script path/name only)
kill_matching_pids "ee_stabilization"
kill_matching_pids "stabilization_vicon_hw.launch"
kill_matching_pids "vicon_relative_bridge"
kill_matching_pids "student_arm.launch.py"
# node binary path fragment unique enough
kill_matching_pids "lib/manipulator/student_arm_node"
sleep 1

arm_claim_serial "${PORT}" || exit 1

STUDENT_PID=""
cleanup() {
  echo "[boot] cleanup" | tee -a "${TEST_LOG}"
  if [[ -n "${STUDENT_PID}" ]]; then
    kill -TERM "${STUDENT_PID}" 2>/dev/null || true
    wait "${STUDENT_PID}" 2>/dev/null || true
  fi
  arm_release_serial || true
}
trap cleanup EXIT INT TERM

STUDENT_CFG="${WS}/install/manipulator/share/manipulator/stabilization_hw_student_arm.yaml"
[[ -f "${STUDENT_CFG}" ]] || STUDENT_CFG="${WS}/src/arm-platform/config/stabilization_hw_student_arm.yaml"
MOTOR_CFG="${WS}/install/manipulator/share/manipulator/motor_config.yaml"
ARM_CFG="${WS}/install/manipulator/share/manipulator/arm_config.yaml"
[[ -f "${MOTOR_CFG}" ]] || MOTOR_CFG="${WS}/src/arm-platform/config/motor_config.yaml"
[[ -f "${ARM_CFG}" ]] || ARM_CFG="${WS}/src/arm-platform/config/arm_config.yaml"

echo "[boot] start student MIT cfg=${STUDENT_CFG}" | tee -a "${TEST_LOG}"
ROS_ARGS=(
  --params-file "${STUDENT_CFG}"
)
if [[ -n "${MIT_GAINS_OVERLAY:-}" ]]; then
  OV="$(readlink -f "${MIT_GAINS_OVERLAY}")"
  if [[ ! -f "${OV}" ]]; then
    echo "[FAIL] MIT_GAINS_OVERLAY not found: ${MIT_GAINS_OVERLAY}" | tee -a "${TEST_LOG}"
    exit 1
  fi
  echo "[boot] gains overlay=${OV}" | tee -a "${TEST_LOG}"
  ROS_ARGS+=(--params-file "${OV}")
fi
ros2 run manipulator student_arm_node --ros-args \
  "${ROS_ARGS[@]}" \
  -p arm_type:=a_l1 \
  -p arm_version:=gamma \
  -p "port_name:=${PORT}" \
  -p "motor_config_path:=${MOTOR_CFG}" \
  -p "arm_config_path:=${ARM_CFG}" \
  -p command_timeout_sec:=2.0 \
  -p max_velocity:=0.5 \
  >"${STUDENT_LOG}" 2>&1 &
ROS2_WRAPPER_PID=$!
# Prefer the real C++ node PID for cleanup
sleep 1
STUDENT_PID="$(pgrep -n -f 'lib/manipulator/student_arm_node' || true)"
if [[ -z "${STUDENT_PID}" ]]; then
  STUDENT_PID="${ROS2_WRAPPER_PID}"
fi
echo "[boot] student_pid=${STUDENT_PID} (wrapper=${ROS2_WRAPPER_PID}) log=${STUDENT_LOG}" | tee -a "${TEST_LOG}"

ok=0
# First discovery can take several seconds; give each probe enough time.
for i in $(seq 1 60); do
  if ! kill -0 "${STUDENT_PID}" 2>/dev/null; then
    echo "[FAIL] student_arm exited early" | tee -a "${TEST_LOG}"
    tail -n 120 "${STUDENT_LOG}" | tee -a "${TEST_LOG}"
    exit 1
  fi
  if timeout 5 ros2 topic echo /joint_states --once >/tmp/mit_js_once.txt 2>/tmp/mit_js_once.err; then
    ok=1
    echo "[boot] /joint_states OK after ~${i} probes" | tee -a "${TEST_LOG}"
    break
  fi
  echo "[boot] waiting joint_states (${i})..." | tee -a "${TEST_LOG}"
  sleep 1
done
if [[ "${ok}" != "1" ]]; then
  echo "[FAIL] no /joint_states within wait window" | tee -a "${TEST_LOG}"
  echo "--- ros2 err ---" | tee -a "${TEST_LOG}"
  cat /tmp/mit_js_once.err 2>/dev/null | tee -a "${TEST_LOG}" || true
  tail -n 120 "${STUDENT_LOG}" | tee -a "${TEST_LOG}"
  exit 1
fi

cd "${PROJECT}"
echo "[boot] run traj all" | tee -a "${TEST_LOG}"
set +e
python3 scripts/hw_mit_traj_test.py --task all --confirm-hw 2>&1 | tee -a "${TEST_LOG}"
rc=${PIPESTATUS[0]}
set -e
echo "[boot] traj exit=${rc}" | tee -a "${TEST_LOG}"
exit "${rc}"
