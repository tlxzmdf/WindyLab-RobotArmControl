#!/usr/bin/env bash
# Mode C 自动调参入口
# 用法:
#   ./run_tune_mode_c.sh auto --max-trials 6
#   ./run_tune_mode_c.sh score --all-recent 5
#   ./run_tune_mode_c.sh apply --kp-pos 700 --osc-lambda 0.04
#   ./run_tune_mode_c.sh --help
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARM_ROOT="$(cd "${PROJECT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
if [[ -f "${ARM_ROOT}/windylab_ws/install/setup.bash" ]]; then
  source "${ARM_ROOT}/windylab_ws/install/setup.bash"
fi

cd "${PROJECT_DIR}"
exec python3 "${PROJECT_DIR}/scripts/mode_c_auto_tune.py" "$@"
