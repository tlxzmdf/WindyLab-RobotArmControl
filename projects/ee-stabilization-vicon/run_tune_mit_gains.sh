#!/usr/bin/env bash
# Auto-tune MIT proximal (j1–j3) kp/kd on hardware.
#
# Examples:
#   ./run_tune_mit_gains.sh apply --kp123 50 --kd123 1.5
#   ./run_tune_mit_gains.sh score --run-dir data/mit_traj/20260717_160251_cosine
#   ./run_tune_mit_gains.sh rank
#   ./run_tune_mit_gains.sh auto --confirm-hw --max-trials 8
#
# After finding a best overlay:
#   MIT_GAINS_OVERLAY=data/tune_mit_gains/prox_j123/overlay_best.yaml ./run_hw_mit_traj_boot.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/../../windylab_ws"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

cd "${ROOT}"
exec python3 scripts/mit_gain_auto_tune.py "$@"
