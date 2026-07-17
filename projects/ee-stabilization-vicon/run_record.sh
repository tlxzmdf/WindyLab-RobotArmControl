#!/usr/bin/env bash
# 真机自稳失效分析录制：在另一终端对正在运行的 run_hw.sh 采集全量数据。
# 用法:
#   ./run_record.sh                          # 录 60 s
#   ./run_record.sh --duration 120 --mode C --note shake --bag
#   DURATION=90 NOTE=hover ./run_record.sh
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARM_ROOT="$(cd "${PROJECT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${ARM_ROOT}/windylab_ws/install/setup.bash"

DURATION="${DURATION:-60}"
MODE="${MODE:-}"
NOTE="${NOTE:-}"
HZ="${HZ:-50}"
POSE_TOPIC="${POSE_TOPIC:-/vrpn/pregme/pose}"

EXTRA=()
if [[ -n "${MODE}" ]]; then EXTRA+=(--mode "${MODE}"); fi
if [[ -n "${NOTE}" ]]; then EXTRA+=(--note "${NOTE}"); fi
if [[ "${BAG:-0}" == "1" || "${BAG:-false}" == "true" ]]; then EXTRA+=(--bag); fi

exec python3 "${PROJECT_DIR}/scripts/record_failure_analysis.py" \
  --duration "${DURATION}" \
  --hz "${HZ}" \
  --pose-topic "${POSE_TOPIC}" \
  "${EXTRA[@]}" \
  "$@"
