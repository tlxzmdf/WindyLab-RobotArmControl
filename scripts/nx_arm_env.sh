#!/usr/bin/env bash
# 机载 Jetson NX 环境加载（由 nx_*.sh 调用）。
# 优先 .nx_arm_env.sh；不存在则用 Humble + 本机工作空间 + ttyTHS3。
# 电脑版请继续用 .pc_arm_env.sh / pc_*.sh，互不影响。

: "${ARM_ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

# shellcheck disable=SC1091
if [[ -f "${ARM_ROOT}/.nx_arm_env.sh" ]]; then
  source "${ARM_ROOT}/.nx_arm_env.sh"
else
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "${ARM_ROOT}/windylab_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ARM_ROOT}/windylab_ws/install/setup.bash"
  fi
  set -u 2>/dev/null || true
  export ARM_ROOT
  export WINDYLAB_WS="${WINDYLAB_WS:-${ARM_ROOT}/windylab_ws}"
  export ARM_DEMO_DIR="${ARM_DEMO_DIR:-${WINDYLAB_WS}/src/arm-platform/demo}"
  export ARM_SERIAL_PORT="${ARM_SERIAL_PORT:-/dev/ttyTHS3}"
  export ARM_MAX_VELOCITY="${ARM_MAX_VELOCITY:-0.2}"
  export ARM_PLATFORM="${ARM_PLATFORM:-nx}"
fi

export ARM_PLATFORM="${ARM_PLATFORM:-nx}"
export ARM_SERIAL_PORT="${ARM_SERIAL_PORT:-/dev/ttyTHS3}"
export ARM_MAX_VELOCITY="${ARM_MAX_VELOCITY:-0.2}"
