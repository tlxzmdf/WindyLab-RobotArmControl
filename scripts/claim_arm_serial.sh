#!/usr/bin/env bash
# 临时占用机载串口（默认 /dev/ttyTHS3）。
# 机载 WindShape robot.service / slave_arm_link_app 等常会占用该口；
# 本脚本在启动教学真机节点前释放占用，退出后尽量恢复。
#
# 用法（由 nx_arm_launch*.sh / 项目 run_hw.sh source）:
#   # shellcheck disable=SC1091
#   source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
#   arm_claim_serial   # 或 arm_claim_serial /dev/ttyTHS3
#   trap 'arm_release_serial' EXIT INT TERM
#
# 环境变量:
#   ARM_CLAIM_SERIAL=0     跳过占用处理
#   ARM_RESTORE_SERIAL=0   退出时不尝试恢复 robot.service
#   ARM_SERIAL_PORT        目标串口

arm_serial_state_file() {
  echo "${ARM_ROOT:-/tmp}/.arm_serial_claim.state"
}

arm_serial_holders() {
  local port="$1"
  # fuser 输出形如: /dev/ttyTHS3:  1234  5678
  fuser "$port" 2>/dev/null | grep -oE '[0-9]+' | sort -u || true
}

arm_describe_pids() {
  local pid
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    if ps -p "$pid" >/dev/null 2>&1; then
      ps -p "$pid" -o pid=,user=,args= 2>/dev/null || echo "$pid (unknown)"
    fi
  done
}

arm_stop_pid_list() {
  local pid
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    if ps -p "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.8
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    if ps -p "$pid" >/dev/null 2>&1; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

# 停止本仓库遗留学生节点（不记入 “需恢复 robot.service”）
arm_stop_own_student_nodes() {
  pkill -f 'student_arm.launch' 2>/dev/null || true
  pkill -f 'student_arm_node' 2>/dev/null || true
  sleep 0.3
}

arm_claim_serial() {
  local port="${1:-${ARM_SERIAL_PORT:-/dev/ttyTHS3}}"
  local state
  state="$(arm_serial_state_file)"

  if [[ "${ARM_CLAIM_SERIAL:-1}" == "0" ]]; then
    echo "[INFO] ARM_CLAIM_SERIAL=0，跳过串口占用处理"
    return 0
  fi

  if [[ ! -e "$port" ]]; then
    echo "[FAIL] 串口不存在: $port"
    return 1
  fi

  arm_stop_own_student_nodes

  local holders
  holders="$(arm_serial_holders "$port")"
  if [[ -z "$holders" ]]; then
    echo "[ OK ] 串口 ${port} 空闲，可直接使用"
    rm -f "$state"
    return 0
  fi

  echo "[WARN] 串口 ${port} 已被占用，将临时释放供本项目使用:"
  # shellcheck disable=SC2086
  arm_describe_pids $holders | sed 's/^/       /'

  local stopped_robot=0
  if systemctl is-active --quiet robot.service 2>/dev/null; then
    echo "[INFO] 检测到 WindShape robot.service 处于 active，优先停止以释放串口…"
    if sudo -n systemctl stop robot.service 2>/dev/null; then
      stopped_robot=1
      echo "[ OK ] 已 stop robot.service（无需密码）"
    elif [[ -t 0 ]] && sudo systemctl stop robot.service; then
      stopped_robot=1
      echo "[ OK ] 已 stop robot.service"
    else
      echo "[WARN] 无法 systemctl stop（无免密或非交互终端），改为结束占用进程"
    fi
    sleep 1
  fi

  holders="$(arm_serial_holders "$port")"
  if [[ -n "$holders" ]]; then
    echo "[INFO] 结束仍占用串口的进程: ${holders}"
    # 若占用方隶属 robot.launch / slave_arm_link，一并清掉同组常见进程
    pkill -f 'slave_arm_link_app' 2>/dev/null || true
    pkill -f 'robot.launch.py' 2>/dev/null || true
    # shellcheck disable=SC2086
    arm_stop_pid_list $holders
    sleep 0.5
  fi

  # robot.service Restart=on-failure 可能把占用进程拉起来；再察看一次
  sleep 1.5
  holders="$(arm_serial_holders "$port")"
  if [[ -n "$holders" ]] && systemctl is-active --quiet robot.service 2>/dev/null; then
    echo "[FAIL] 串口再次被 robot.service 占用（服务会自动重启链路）。"
    echo "       请先手动停止后再重试本脚本:"
    echo "         sudo systemctl stop robot.service"
    echo "       （建议配置对该命令的 NOPASSWD，以便脚本自动 stop/start）"
    return 1
  fi

  holders="$(arm_serial_holders "$port")"
  if [[ -n "$holders" ]]; then
    echo "[FAIL] 仍无法释放串口 ${port}，占用 PID: ${holders}"
    # shellcheck disable=SC2086
    arm_describe_pids $holders | sed 's/^/       /'
    echo "       可手动: fuser -v ${port}  或  sudo systemctl stop robot.service"
    return 1
  fi

  {
    echo "port=${port}"
    echo "stopped_robot=${stopped_robot}"
    echo "claimed_at=$(date -Iseconds)"
  } >"$state"
  echo "[ OK ] 已临时占用 ${port}（退出本程序后将尝试归还）"
  return 0
}

arm_release_serial() {
  local state
  state="$(arm_serial_state_file)"

  if [[ "${ARM_RESTORE_SERIAL:-1}" == "0" ]]; then
    echo "[INFO] ARM_RESTORE_SERIAL=0，不自动恢复 robot.service"
    return 0
  fi

  if [[ ! -f "$state" ]]; then
    return 0
  fi

  local stopped_robot=0
  # shellcheck disable=SC1090
  stopped_robot="$(grep -E '^stopped_robot=' "$state" | cut -d= -f2 || echo 0)"

  # 先确保本项目节点已退出，避免占口导致无法恢复
  arm_stop_own_student_nodes

  if [[ "$stopped_robot" == "1" ]]; then
    echo "[INFO] 尝试恢复 WindShape robot.service…"
    if sudo -n systemctl start robot.service 2>/dev/null; then
      echo "[ OK ] 已重新 start robot.service"
    elif [[ -t 0 ]] && sudo systemctl start robot.service; then
      echo "[ OK ] 已重新 start robot.service"
    else
      echo "[WARN] 未能自动恢复 robot.service，请手动执行:"
      echo "       sudo systemctl start robot.service"
    fi
  else
    echo "[INFO] 本次未通过 robot.service 接管串口；若机载链路异常，可手动:"
    echo "       sudo systemctl restart robot.service"
  fi

  rm -f "$state"
}
