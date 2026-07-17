#!/usr/bin/env bash
# Hardware MIT trajectory test (cosine / line / circle).
# Does NOT start student_arm — bring MIT up yourself first.
# Does NOT kill processes unless you pass --kill-stab.
#
# Prerequisites:
#   1. student_arm with controller_type:=mit_stabilization on the serial port
#   2. No ee_stabilization publishing /student/joint_command
#   3. Arm clear of obstacles; defaults match student demos
#      (line/cosine ≈0.316 m, circle r=0.08 m, plane=YZ)
#
# Examples:
#   ./run_hw_mit_traj.sh cosine
#   ./run_hw_mit_traj.sh line
#   ./run_hw_mit_traj.sh circle
#   ./run_hw_mit_traj.sh all
#   ./run_hw_mit_traj.sh plot data/mit_traj/20260717_120000_cosine
#
# Extra args after the task are forwarded to the Python script, e.g.:
#   ./run_hw_mit_traj.sh cosine --amplitude 0.03 --axis y

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/../../windylab_ws"
TASK="${1:-}"
shift || true

if [[ -z "${TASK}" ]]; then
  echo "usage: $0 {cosine|line|circle|all|plot <run-dir>} [extra args...]" >&2
  exit 2
fi

# Optional: stop competing stabilization (explicit only)
if [[ "${KILL_STAB:-0}" == "1" ]] || [[ "${1:-}" == "--kill-stab" ]]; then
  if [[ "${1:-}" == "--kill-stab" ]]; then shift; fi
  echo "[run_hw_mit_traj] stopping ee_stabilization / launch leftovers..."
  pkill -TERM -f 'stabilization_vicon_hw.launch' 2>/dev/null || true
  pkill -TERM -f 'arm_ee_stabilization_control/ee_stabilization' 2>/dev/null || true
  sleep 1
fi

# ROS env
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi

cd "${ROOT}"

if [[ "${TASK}" == "plot" ]]; then
  RUN_DIR="${1:-}"
  shift || true
  if [[ -z "${RUN_DIR}" ]]; then
    echo "usage: $0 plot <run-dir>" >&2
    exit 2
  fi
  exec python3 scripts/hw_mit_traj_test.py --plot-only --run-dir "${RUN_DIR}" "$@"
fi

case "${TASK}" in
  cosine|line|circle|all) ;;
  *)
    echo "unknown task: ${TASK}" >&2
    exit 2
    ;;
esac

echo "[run_hw_mit_traj] task=${TASK}"
echo "[run_hw_mit_traj] publishing to /student/joint_command (MIT position track)"
exec python3 scripts/hw_mit_traj_test.py --task "${TASK}" --confirm-hw "$@"
