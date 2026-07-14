#!/usr/bin/env bash
# 兼容入口：转发到 projects/ee-stabilization/run_sim.sh
set -eo pipefail
exec "$(cd "$(dirname "$0")" && pwd)/projects/ee-stabilization/run_sim.sh" "$@"
