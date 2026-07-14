#!/usr/bin/env bash
# 电脑版（WSL2 + USB 串口）真机一键：检测环境 → 安装依赖 → 编译 → 串口测试 → 输出启动指引
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ARM_ROOT}/windylab_ws"
DEMO_DIR="${WS}/src/arm-platform/demo"
ENV_FILE="${ARM_ROOT}/.pc_arm_env.sh"
SERIAL_PORT="${ARM_SERIAL_PORT:-/dev/ttyUSB0}"
BAUD=921600
SKIP_BUILD=0
FORCE_BUILD=0
SKIP_SERIAL=0
FAILURES=0

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC} $*"; }
ok()    { echo -e "${GREEN}[ OK ]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail()  { echo -e "${RED}[FAIL]${NC} $*"; FAILURES=$((FAILURES + 1)); }

usage() {
  cat <<EOF
用法: $0 [选项]

  电脑版真机一键配置（WSL2 + /dev/ttyUSB0）。
  完成后按屏幕指引开两个终端即可让机械臂动起来。

选项:
  --force-build       强制重新 colcon 编译（默认：已编译则自动跳过）
  --skip-build        同默认跳过行为（兼容旧用法）
  --skip-serial       跳过串口数据检测（机械臂未上电时）
  --port <path>       串口路径，默认 /dev/ttyUSB0
  -h, --help          显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --force-build) FORCE_BUILD=1; shift ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --skip-serial) SKIP_SERIAL=1; shift ;;
    --port) SERIAL_PORT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "未知参数: $1"; usage; exit 1 ;;
  esac
done

banner() {
  cat <<'EOF'

╔══════════════════════════════════════════════════════════════╗
║  A-L1 机械臂 — 电脑版真机一键配置 (WSL2 + USB)               ║
╚══════════════════════════════════════════════════════════════╝

EOF
}

step() {
  echo ""
  echo -e "${CYAN}━━━ $* ━━━${NC}"
}

check_wsl() {
  step "1/7 运行环境"
  if grep -qi microsoft /proc/version 2>/dev/null; then
    ok "检测到 WSL 环境"
  else
    warn "未检测到 WSL（若在原生 Ubuntu 上也可继续，串口可能是 ttyTHS3）"
  fi
  if [[ -d "${WS}" ]]; then
    ok "工作空间: ${WS}"
  else
    fail "未找到 windylab_ws: ${WS}"
    return 1
  fi
}

check_ros() {
  step "2/7 ROS 2 Humble"
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    ok "ROS 2 Humble 已安装"
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u 2>/dev/null || true
  else
    fail "未找到 /opt/ros/humble/setup.bash"
    echo "  请先安装 ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html"
    return 1
  fi
}

install_deps() {
  step "3/7 系统与 Python 依赖"
  local pkgs=(
    ros-humble-robot-state-publisher
    ros-humble-rviz2
    ros-humble-pinocchio
    libeigen3-dev
    python3-pip
    python3-serial
  )
  local missing=()
  for p in "${pkgs[@]}"; do
    if ! dpkg -s "$p" &>/dev/null; then
      missing+=("$p")
    fi
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    info "安装缺失 apt 包: ${missing[*]}"
    if sudo apt-get update -qq && sudo apt-get install -y "${missing[@]}"; then
      ok "apt 依赖安装完成"
    else
      fail "apt 安装失败，请手动: sudo apt install ${missing[*]}"
    fi
  else
    ok "apt 依赖已满足"
  fi

  if python3 -c "import numpy" 2>/dev/null; then
    ok "numpy 已安装"
  else
    info "安装 numpy..."
    pip3 install -q numpy && ok "numpy 安装完成" || fail "pip3 install numpy 失败"
  fi

  if python3 -c "import serial" 2>/dev/null; then
    ok "pyserial 已安装"
  else
    info "安装 pyserial..."
    pip3 install -q pyserial && ok "pyserial 安装完成" || fail "pip3 install pyserial 失败"
  fi
}

setup_serial_permission() {
  step "4/7 串口权限"
  if groups "$USER" 2>/dev/null | grep -q dialout; then
    ok "用户 ${USER} 已在 dialout 组"
  else
    warn "将 ${USER} 加入 dialout 组（需要输入 sudo 密码）"
    if sudo usermod -aG dialout "$USER"; then
      ok "已加入 dialout 组"
      warn "若仍无法打开串口，请关闭并重新打开 WSL 终端后再试"
    else
      fail "usermod dialout 失败；可临时用 sudo 运行 launch"
    fi
  fi
}

workspace_is_built() {
  [[ -f "${WS}/install/setup.bash" ]] || return 1
  local node_bin="${WS}/install/manipulator/lib/manipulator/student_arm_node"
  [[ -x "${node_bin}" ]] || return 1
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
  set -u 2>/dev/null || true
  ros2 pkg prefix manipulator &>/dev/null
}

build_workspace() {
  step "5/7 编译工作空间"
  cd "${WS}"

  if [[ "$FORCE_BUILD" -eq 1 ]]; then
    info "强制重新编译 (--force-build) ..."
    if colcon build --symlink-install --packages-up-to manipulator; then
      ok "编译完成"
    else
      warn "colcon build 报错"
    fi
  elif [[ "$SKIP_BUILD" -eq 1 ]] || workspace_is_built; then
    if workspace_is_built; then
      ok "检测到已编译的 manipulator，跳过 colcon build（需重编请加 --force-build）"
    else
      warn "跳过编译 (--skip-build)，但工作空间可能不完整"
    fi
  else
    info "未检测到完整编译产物，开始 colcon build ..."
    if colcon build --symlink-install --packages-up-to manipulator; then
      ok "编译完成"
    else
      warn "colcon build 报错"
    fi
  fi

  if [[ -f "${WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${WS}/install/setup.bash"
    set -u 2>/dev/null || true
    if ros2 pkg prefix manipulator &>/dev/null; then
      ok "manipulator 包可用"
    else
      fail "找不到 manipulator 包，请修复编译错误后重试"
      return 1
    fi
  else
    fail "缺少 ${WS}/install/setup.bash，请先成功编译"
    return 1
  fi
}

write_env_and_helpers() {
  step "6/7 写入环境配置与快捷脚本"

  cat > "${ENV_FILE}" <<EOF
# 由 pc_real_arm_setup.sh 生成 — 电脑版真机环境
export ARM_ROOT="${ARM_ROOT}"
export WINDYLAB_WS="${WS}"
export ARM_DEMO_DIR="${DEMO_DIR}"
export ARM_SERIAL_PORT="${SERIAL_PORT}"
export ARM_MAX_VELOCITY="${ARM_MAX_VELOCITY:-0.2}"

source /opt/ros/humble/setup.bash
source "\${WINDYLAB_WS}/install/setup.bash"
EOF
  ok "环境文件: ${ENV_FILE}"

  # bashrc 自动加载（幂等）
  local marker="# >>> arm-pc-real-env >>>"
  if ! grep -qF "${marker}" "${HOME}/.bashrc" 2>/dev/null; then
    cat >> "${HOME}/.bashrc" <<EOF

${marker}
if [[ -f "${ENV_FILE}" ]]; then source "${ENV_FILE}"; fi
# <<< arm-pc-real-env <<<
EOF
    ok "已追加到 ~/.bashrc（新终端自动加载 ROS + 工作空间）"
  else
    ok "~/.bashrc 已包含 arm 环境配置"
  fi

  set +u
  # shellcheck disable=SC1091
  source "${ENV_FILE}"
  set -u 2>/dev/null || true

  # 终端 1：launch
  cat > "${ARM_ROOT}/pc_arm_launch.sh" <<'LAUNCH_EOF'
#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
  echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在。"
  echo "  Windows PowerShell(管理员): usbipd list && usbipd attach --wsl --busid <BUSID>"
  exit 1
fi

echo "启动真机 launch: port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=False
LAUNCH_EOF

  # 终端 1：仿真 launch
  cat > "${ARM_ROOT}/pc_arm_launch_sim.sh" <<'SIM_LAUNCH_EOF'
#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

echo "启动仿真 launch (arm_type:=sim, use_rviz:=False)"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=sim \
  use_rviz:=False
SIM_LAUNCH_EOF

  # 终端 1：真机 launch + RViz
  cat > "${ARM_ROOT}/pc_arm_launch_rviz.sh" <<'RVIZ_LAUNCH_EOF'
#!/usr/bin/env bash
# 电脑专用版 · 真机 launch + RViz 可视化（终端 1）
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

pkill -f "student_arm.launch" 2>/dev/null || true
pkill -f student_arm_node 2>/dev/null || true
sleep 0.3

if [[ ! -e "${ARM_SERIAL_PORT}" ]]; then
  echo "[FAIL] 串口 ${ARM_SERIAL_PORT} 不存在。"
  echo "  Windows PowerShell(管理员): usbipd list && usbipd attach --wsl --busid <BUSID>"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]] && [[ ! -e /tmp/.X11-unix/X0 ]] && [[ ! -d /mnt/wslg ]]; then
  echo "[WARN] 未检测到图形环境。RViz 需要 WSLg(Win11) 或 X11。"
  echo "       若窗口无法弹出，可改用: ./pc_arm_launch.sh (无 RViz)"
fi

echo "启动真机 launch + RViz: port=${ARM_SERIAL_PORT} max_velocity=${ARM_MAX_VELOCITY}"
exec ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${ARM_SERIAL_PORT}" \
  max_velocity:="${ARM_MAX_VELOCITY}" \
  use_rviz:=True
RVIZ_LAUNCH_EOF

  # 终端 2：demo
  cat > "${ARM_ROOT}/pc_arm_demo.sh" <<'DEMO_EOF'
#!/usr/bin/env bash
set -eo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ARM_ROOT}/.pc_arm_env.sh"

DEMO="${1:-move_arm_demo.py}"
cd "${ARM_DEMO_DIR}"
if [[ ! -f "${DEMO}" ]]; then
  echo "[FAIL] Demo 不存在: ${ARM_DEMO_DIR}/${DEMO}"
  echo "  可用: move_arm_demo.py move_arm_ik_demo.py move_arm_line_demo.py rotate_link5_right_90.py"
  exit 1
fi
echo "运行 Demo: ${DEMO}"
exec python3 "${DEMO}"
DEMO_EOF

  chmod +x "${ARM_ROOT}/pc_arm_launch.sh" "${ARM_ROOT}/pc_arm_launch_sim.sh" "${ARM_ROOT}/pc_arm_launch_rviz.sh" "${ARM_ROOT}/pc_arm_demo.sh"
  ok "快捷脚本: pc_arm_launch_sim.sh (终端1·仿真)  pc_arm_launch.sh (终端1·真机)  pc_arm_launch_rviz.sh (终端1·真机+RViz)  pc_arm_demo.sh (终端2)"
}

detect_serial_port() {
  if [[ -e "${SERIAL_PORT}" ]]; then
    return 0
  fi
  local first
  first="$(ls /dev/ttyUSB* 2>/dev/null | head -1 || true)"
  if [[ -n "${first}" ]]; then
  SERIAL_PORT="${first}"
    warn "默认 /dev/ttyUSB0 不存在，改用 ${SERIAL_PORT}"
    # 更新 env 中的端口
    sed -i "s|^export ARM_SERIAL_PORT=.*|export ARM_SERIAL_PORT=\"${SERIAL_PORT}\"|" "${ENV_FILE}" 2>/dev/null || true
    return 0
  fi
  return 1
}

test_serial() {
  step "7/7 串口与机械臂通信"
  if [[ "$SKIP_SERIAL" -eq 1 ]]; then
    warn "跳过串口测试 (--skip-serial)"
    return 0
  fi

  if ! detect_serial_port; then
    fail "未找到 USB 串口设备 (/dev/ttyUSB*)"
    cat <<'USBIPD'

  ┌─ Windows 侧（PowerShell 管理员）────────────────────────────┐
  │  winget install usbipd                                       │
  │  usbipd list                    # 找到 CP2102 / Silicon Labs │
  │  usbipd bind --busid <BUSID>                                 │
  │  usbipd attach --wsl --busid <BUSID>                         │
  └──────────────────────────────────────────────────────────────┘
  然后在 WSL 中重新运行:  ./pc_real_arm_setup.sh

USBIPD
    return 1
  fi

  ok "串口设备: ${SERIAL_PORT}"
  ls -la "${SERIAL_PORT}"

  pkill -f student_arm_node 2>/dev/null || true
  sleep 0.2

  info "监听 4 秒，确认机械臂是否上电并发送数据..."
  if python3 "${WS}/tools/serial_sniff.py" --port "${SERIAL_PORT}" --baud "${BAUD}" --duration 4; then
    ok "串口通信正常"
  else
    local rc=$?
    if [[ $rc -eq 2 ]]; then
      fail "串口可打开但 4 秒内无数据"
      echo "  请确认: ①机械臂已上电  ②USB 转串口 TX/RX 已接控制板  ③线缆牢固"
      echo "  若仅暂时未接机械臂，可: $0 --skip-serial"
    else
      fail "串口测试失败 (exit ${rc})"
    fi
    return 1
  fi
}

print_next_steps() {
  echo ""
  if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  配置完成 — 按下面两步即可让机械臂动起来                    ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
  else
    echo -e "${YELLOW}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║  配置未完全通过 (${FAILURES} 项失败)，请先处理上方 [FAIL]       ║${NC}"
    echo -e "${YELLOW}╚══════════════════════════════════════════════════════════════╝${NC}"
  fi

  cat <<EOF

【电脑 + 真机 — 快捷脚本】

  终端 1:  cd ${ARM_ROOT} && ./pc_arm_launch.sh
  终端 2:  cd ${ARM_ROOT} && ./pc_arm_demo.sh

【电脑 + 真机 + RViz — 真机运动 + 屏幕显示模型（需 WSLg/X11）】

  终端 1:  cd ${ARM_ROOT} && ./pc_arm_launch_rviz.sh
  终端 2:  cd ${ARM_ROOT} && ./pc_arm_demo.sh

【电脑 + 仿真 — 快捷脚本（无需 USB）】

  终端 1:  cd ${ARM_ROOT} && ./pc_arm_launch_sim.sh
  终端 2:  cd ${ARM_ROOT} && ./pc_arm_demo.sh

  详见: ${ARM_ROOT}/详细使用手册.md  第 5 节

【方式 B — 电脑真机手动命令】

  终端 1:
    source ${ENV_FILE}
    ros2 launch manipulator student_arm.launch.py \\
      arm_type:=a_l1 port_name:=${SERIAL_PORT} max_velocity:=0.2 use_rviz:=False

  终端 2:
    source ${ENV_FILE}
    cd ${DEMO_DIR}
    python3 move_arm_demo.py

【验证】终端 2 另开窗口:
    source ${ENV_FILE}
    ros2 topic echo /joint_states --once
    # position 不应长期全为 0

【安全】真机首次测试请确认周围无人；异常立即 Ctrl+C 并断电。
【文档】${ARM_ROOT}/详细使用手册.md

EOF
}

main() {
  banner
  check_wsl || true
  check_ros || { print_next_steps; exit 1; }
  install_deps || true
  setup_serial_permission || true
  build_workspace || true
  write_env_and_helpers
  test_serial || true
  print_next_steps
  [[ $FAILURES -eq 0 ]]
}

main "$@"
