#!/usr/bin/env bash
# 机载 NX · 运行 arm-platform Demo（终端 2）
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/nx_arm_env.sh"

DEMO="${1:-move_arm_demo.py}"
cd "${ARM_DEMO_DIR}"
if [[ ! -f "${DEMO}" ]]; then
  echo "[FAIL] Demo 不存在: ${ARM_DEMO_DIR}/${DEMO}"
  echo "  可用: move_arm_demo.py move_arm_ik_demo.py move_arm_line_demo.py rotate_link5_right_90.py"
  exit 1
fi
echo "运行 Demo (NX): ${DEMO}"
exec python3 "${DEMO}"
