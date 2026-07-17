#!/usr/bin/env bash
# Plot latest (or named) Mode B failure-analysis run — continuous curves.
# Usage:
#   ./run_plot_mode_b.sh
#   ./run_plot_mode_b.sh data/runs/20260717_185428_B_outerfix_1min
#   TITLE=OuterFix ./run_plot_mode_b.sh --latest
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARGS=("$@")
if [[ ${#ARGS[@]} -eq 0 ]]; then
  ARGS=(--latest)
elif [[ ${#ARGS[@]} -eq 1 && "${ARGS[0]}" != --* ]]; then
  ARGS=(--run "${ARGS[0]}")
fi
if [[ -n "${TITLE:-}" ]]; then
  ARGS+=(--title-prefix "${TITLE}")
fi
exec python3 "${PROJECT_DIR}/scripts/plot_mode_b_run.py" "${ARGS[@]}"
