#!/usr/bin/env bash
# 真机只读观察：订阅已有 /joint_states + TF，不启动 student_arm，不 pkill 任何进程。
#
# 用法（真机已在运行时另开终端）:
#   终端 1: cd /root/arm && ARM_MAX_VELOCITY=0.35 ./pc_arm_launch.sh
#   终端 2: python3 circle_draw_node.py ...   （或其它控制）
#   终端 3: ./observe_hw.sh
#
# 与 run_sim.sh 的区别：run_sim.sh 会 pkill 并启动 arm_type:=sim，不能与真机同时用。
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

TRAIL_DELAY="${TRAIL_DELAY:-10}"
TRAIL_DURATION="${TRAIL_DURATION:-10}"
WITH_TRAIL="${WITH_TRAIL:-1}"

if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -d /mnt/wslg ]]; then
    export DISPLAY=:0
  else
    cat <<'EOF'
[错误] 未设置 DISPLAY，无法打开 RViz。
  WSLg (Windows 11) 或配置 X11 后再运行 observe_hw.sh
EOF
    exit 1
  fi
fi

if ! pgrep -f student_arm_node >/dev/null 2>&1; then
  echo "[错误] 未检测到 student_arm_node。"
  echo "  请先在另一终端启动真机: cd /root/arm && ARM_MAX_VELOCITY=0.35 ./pc_arm_launch.sh"
  exit 1
fi

echo "[observe_hw] 等待 /joint_states（来自真机 student_arm）..."
ready=0
for _ in $(seq 1 50); do
  if ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.2
done
if [[ "${ready}" != "1" ]]; then
  echo "[错误] 未收到 /joint_states"
  exit 1
fi

# 确认是真机而非仿真（sim 时也可观察，但提示用户）
if ros2 param get /student_arm_node arm_type 2>/dev/null | grep -q "sim"; then
  echo "[observe_hw] 警告: 当前 student_arm 为 sim 模式，不是真机。"
fi

cat <<EOF

╔══════════════════════════════════════════════════════════╗
║  真机只读观察 (observe_hw)                               ║
╠══════════════════════════════════════════════════════════╣
║  · 不启动 student_arm，不 pkill，不影响真机控制          ║
║  · RViz 模型来自真机 /joint_states + TF                  ║
║  · 橙色小球 = 当前末端 link7                             ║
║  · 橙色轨迹 = 延迟 ${TRAIL_DELAY}s（WITH_TRAIL=1 时）    ║
╚══════════════════════════════════════════════════════════╝

EOF

VIZ_PID=""
if [[ "${WITH_TRAIL}" == "1" ]]; then
  python3 "${PROJECT_DIR}/scripts/ee_trajectory_viz.py" \
    --delay "${TRAIL_DELAY}" --trail-duration "${TRAIL_DURATION}" &
  VIZ_PID=$!
fi

echo "[observe_hw] 启动 RViz ..."
rviz2 -d "${PROJECT_DIR}/hw_observe.rviz" &
RVIZ_PID=$!

cleanup() {
  [[ -n "${VIZ_PID}" ]] && kill "${VIZ_PID}" 2>/dev/null || true
  kill "${RVIZ_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[observe_hw] 观察窗口已打开。Ctrl+C 退出（真机与控制节点不受影响）。"
wait
