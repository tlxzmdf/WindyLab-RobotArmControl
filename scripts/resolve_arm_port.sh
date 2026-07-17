#!/usr/bin/env bash
# 解析真机串口默认值（不覆盖调用方已设置的变量）。
# 优先级: PORT_NAME > ARM_SERIAL_PORT > ARM_PLATFORM=nx|pc > Jetson 自动检测 > /dev/ttyUSB0
#
# 用法（在项目脚本中）:
#   ARM_ROOT=...
#   # shellcheck disable=SC1091
#   source "${ARM_ROOT}/scripts/resolve_arm_port.sh"
#   PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
#
# 电脑用法保持不变：非 Jetson 默认仍为 /dev/ttyUSB0；也可显式:
#   ARM_PLATFORM=pc PORT_NAME=/dev/ttyUSB0 ./run_hw.sh
# NX:
#   ARM_PLATFORM=nx ./run_hw.sh   # 或自动检测 Tegra → /dev/ttyTHS3

if [[ -n "${PORT_NAME:-}" ]]; then
  export ARM_SERIAL_PORT="${PORT_NAME}"
  return 0 2>/dev/null || true
fi

if [[ -n "${ARM_SERIAL_PORT:-}" ]]; then
  return 0 2>/dev/null || true
fi

_is_jetson() {
  [[ -f /etc/nv_tegra_release ]] && return 0
  [[ -e /sys/module/tegra_fuse ]] && return 0
  grep -qiE 'tegra|jetson|nvidia' /proc/device-tree/model 2>/dev/null && return 0
  return 1
}

case "${ARM_PLATFORM:-}" in
  nx|NX|jetson|JETSON)
    export ARM_SERIAL_PORT="/dev/ttyTHS3"
    ;;
  pc|PC|wsl|WSL)
    export ARM_SERIAL_PORT="/dev/ttyUSB0"
    ;;
  *)
    if _is_jetson; then
      export ARM_SERIAL_PORT="/dev/ttyTHS3"
    else
      export ARM_SERIAL_PORT="/dev/ttyUSB0"
    fi
    ;;
esac
