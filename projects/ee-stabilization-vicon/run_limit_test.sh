#!/usr/bin/env bash
# 扰动极限测试：幅度 × 频率扫参，三模式 A/B/C
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source "$ROOT/../../windylab_ws/install/setup.bash"

PHASE="${1:-quick}"
shift || true

python3 "$ROOT/scripts/run_limit_test.py" --phase "$PHASE" "$@"
