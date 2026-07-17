#!/usr/bin/env bash
# 机载 Jetson NX 真机一键：检测环境 → 依赖 → 编译 → 串口自检 → 输出启动指引
# 电脑版请继续用 ./pc_real_arm_setup.sh（互不影响）
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ARM_ROOT}/windylab_ws"
DEMO_DIR="${WS}/src/arm-platform/demo"
ENV_FILE="${ARM_ROOT}/.nx_arm_env.sh"
SERIAL_PORT="${ARM_SERIAL_PORT:-/dev/ttyTHS3}"
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

  机载 NX 真机一键配置（Ubuntu 22.04 + ROS 2 Humble + /dev/ttyTHS3）。
  电脑 / WSL 请用: ./pc_real_arm_setup.sh

选项:
  --force-build       强制重新 colcon 编译
  --skip-build        跳过编译
  --skip-serial       跳过串口数据检测（机械臂未上电时）
  --port <path>       串口路径，默认 /dev/ttyTHS3
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
║  A-L1 机械臂 — 机载 NX 真机一键配置 (ttyTHS3)                ║
╚══════════════════════════════════════════════════════════════╝

EOF
}

step() {
  echo ""
  echo -e "${CYAN}━━━ $* ━━━${NC}"
}

check_platform() {
  step "1/7 运行环境"
  if [[ -f /etc/nv_tegra_release ]] || [[ -e /sys/module/tegra_fuse ]]; then
    ok "检测到 Jetson / Tegra 平台"
  else
    warn "未检测到 Tegra 特征；若确为本机原生 Ubuntu 机载环境可继续"
  fi
  info "架构: $(uname -m)  工作区: ${ARM_ROOT}"
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
    warn "将 ${USER} 加入 dialout 组（需要 sudo）"
    if sudo usermod -aG dialout "$USER"; then
      ok "已加入 dialout 组 — 请重新登录后再开串口"
    else
      fail "usermod dialout 失败"
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
    colcon build --symlink-install --packages-up-to manipulator && ok "编译完成" || warn "colcon build 报错"
  elif [[ "$SKIP_BUILD" -eq 1 ]] || workspace_is_built; then
    if workspace_is_built; then
      ok "已检测到 manipulator，跳过 colcon build（需重编请加 --force-build）"
    else
      warn "跳过编译 (--skip-build)，工作空间可能不完整"
    fi
  else
    info "开始 colcon build ..."
    colcon build --symlink-install --packages-up-to manipulator && ok "编译完成" || warn "colcon build 报错"
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
    fail "缺少 ${WS}/install/setup.bash"
    return 1
  fi
}

check_meshes_symlink() {
  local mesh_link="${WS}/src/arm_ee_stabilization_description/meshes"
  if [[ -L "${mesh_link}" ]] && [[ ! -e "${mesh_link}" ]]; then
    warn "arm_ee_stabilization_description/meshes 软链接断开，机头稳定可视化可能失败"
    warn "可改指向: ln -sfn ../dummy_description/meshes ${mesh_link}"
  fi
}

write_env() {
  step "6/7 写入 NX 环境配置"

  cat > "${ENV_FILE}" <<EOF
# 由 nx_real_arm_setup.sh 生成 — 机载 NX 真机环境
export ARM_PLATFORM=nx
export ARM_ROOT="${ARM_ROOT}"
export WINDYLAB_WS="${WS}"
export ARM_DEMO_DIR="${DEMO_DIR}"
export ARM_SERIAL_PORT="${SERIAL_PORT}"
export ARM_MAX_VELOCITY="${ARM_MAX_VELOCITY:-0.2}"

source /opt/ros/humble/setup.bash
source "\${WINDYLAB_WS}/install/setup.bash"
EOF
  ok "环境文件: ${ENV_FILE}"

  local marker="# >>> arm-nx-real-env >>>"
  if ! grep -qF "${marker}" "${HOME}/.bashrc" 2>/dev/null; then
    cat >> "${HOME}/.bashrc" <<EOF

${marker}
if [[ -f "${ENV_FILE}" ]]; then source "${ENV_FILE}"; fi
# <<< arm-nx-real-env <<<
EOF
    ok "已追加到 ~/.bashrc（新终端自动加载）"
  else
    ok "~/.bashrc 已包含 NX arm 环境配置"
  fi

  chmod +x \
    "${ARM_ROOT}/nx_arm_launch.sh" \
    "${ARM_ROOT}/nx_arm_launch_sim.sh" \
    "${ARM_ROOT}/nx_arm_launch_rviz.sh" \
    "${ARM_ROOT}/nx_arm_demo.sh" \
    "${ARM_ROOT}/nx_arm_record_demo.sh" \
    "${ARM_ROOT}/nx_real_arm_setup.sh" \
    "${ARM_ROOT}/scripts/nx_arm_env.sh" \
    "${ARM_ROOT}/scripts/resolve_arm_port.sh" 2>/dev/null || true

  ok "快捷脚本: nx_arm_launch_sim.sh / nx_arm_launch.sh / nx_arm_launch_rviz.sh / nx_arm_demo.sh"
}

detect_serial_port() {
  if [[ -e "${SERIAL_PORT}" ]]; then
    return 0
  fi
  local first
  first="$(ls /dev/ttyTHS3 /dev/ttyTHS* /dev/ttyUSB* 2>/dev/null | head -1 || true)"
  if [[ -n "${first}" ]]; then
    SERIAL_PORT="${first}"
    warn "默认串口不存在，改用 ${SERIAL_PORT}"
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
    fail "未找到串口设备（期望 /dev/ttyTHS3）"
    echo "  请确认按 SOP 将机械臂 UART 接到 NX，并检查: ls -la /dev/ttyTHS*"
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
      echo "  请确认: ①机械臂已上电  ②接线符合 SOP  ③端口未被其他进程占用"
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

【机载 NX + 真机 — 快捷脚本】

  终端 1:  cd ${ARM_ROOT} && ./nx_arm_launch.sh
  终端 2:  cd ${ARM_ROOT} && ./nx_arm_demo.sh

【机载 NX + 仿真】

  终端 1:  cd ${ARM_ROOT} && ./nx_arm_launch_sim.sh
  终端 2:  cd ${ARM_ROOT} && ./nx_arm_demo.sh

【机载 NX + 真机 + RViz】（需图形环境；无屏请用 nx_arm_launch.sh）

  终端 1:  cd ${ARM_ROOT} && ./nx_arm_launch_rviz.sh
  终端 2:  cd ${ARM_ROOT} && ./nx_arm_demo.sh

【手动命令】

  source ${ENV_FILE}
  ros2 launch manipulator student_arm.launch.py \\
    arm_type:=a_l1 port_name:=${SERIAL_PORT} max_velocity:=0.2 use_rviz:=False

【电脑版】请用 ./pc_real_arm_setup.sh 与 pc_arm_*.sh（串口默认 /dev/ttyUSB0）
【文档】${ARM_ROOT}/详细使用手册.md 、 ${ARM_ROOT}/STUDENT_GUIDE.md

EOF
}

main() {
  banner
  check_platform || true
  check_ros || { print_next_steps; exit 1; }
  install_deps || true
  setup_serial_permission || true
  build_workspace || true
  check_meshes_symlink || true
  write_env
  test_serial || true
  print_next_steps
  [[ $FAILURES -eq 0 ]]
}

main "$@"
