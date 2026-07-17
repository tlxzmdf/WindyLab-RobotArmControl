#!/usr/bin/env bash
# 真机画圆 (circle-draw) + 数据录制 → ~/arm/run_data/<时间戳>_circle_draw_<mode>/
set -eo pipefail

ARM_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 保留调用方传入的限速（circle-draw 推荐 0.35，勿被 .pc_arm_env.sh 覆盖）
_CLI_MAX_VEL="${ARM_MAX_VELOCITY:-}"

# Prefer NX env on Jetson; keep PC env for WSL/laptop
# shellcheck disable=SC1091
if [[ -f /etc/nv_tegra_release || -e /sys/module/tegra_fuse || "${ARM_PLATFORM:-}" == "nx" ]] \
  && [[ -f "${ARM_ROOT}/.nx_arm_env.sh" ]]; then
  source "${ARM_ROOT}/.nx_arm_env.sh"
elif [[ -f "${ARM_ROOT}/.pc_arm_env.sh" ]]; then
  source "${ARM_ROOT}/.pc_arm_env.sh"
else
  source /opt/ros/humble/setup.bash
  source "${ARM_ROOT}/windylab_ws/install/setup.bash"
fi

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/resolve_arm_port.sh"

MODE="${1:-diff}"
DURATION="${2:-24}"
MAX_VEL="${_CLI_MAX_VEL:-0.35}"
PORT="${PORT_NAME:-${ARM_SERIAL_PORT}}"
RECORD_PY="${WINDYLAB_WS:-${ARM_ROOT}/windylab_ws}/tools/record_demo_run.py"
PLOT_PY="${WINDYLAB_WS:-${ARM_ROOT}/windylab_ws}/tools/plot_demo_run.py"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ARM_ROOT}/run_data/${STAMP}_circle_draw_${MODE}"
DEMO_LABEL="circle_draw_node.py --mode ${MODE}"

if [[ ! -e "${PORT}" ]]; then
  echo "[FAIL] 串口 ${PORT} 不存在，请先 usbipd attach"
  exit 1
fi

mkdir -p "${ARM_ROOT}/run_data"

# shellcheck disable=SC1091
source "${ARM_ROOT}/scripts/claim_arm_serial.sh"
arm_claim_serial "${PORT}" || exit 1

echo "[INFO] 真机 circle-draw mode=${MODE} duration=${DURATION}s max_vel=${MAX_VEL} port=${PORT}"
echo "[INFO] 输出: ${OUT_DIR}"

ros2 launch manipulator student_arm.launch.py \
  arm_type:=a_l1 \
  port_name:="${PORT}" \
  max_velocity:="${MAX_VEL}" \
  use_rviz:=False &
LAUNCH_PID=$!

cleanup() {
  kill "${DEMO_PID:-}" 2>/dev/null || true
  kill "${RECORD_PID:-}" 2>/dev/null || true
  kill "${LAUNCH_PID:-}" 2>/dev/null || true
  pkill -f '[s]tudent_arm.launch' 2>/dev/null || true
  pkill -f '[s]tudent_arm_node' 2>/dev/null || true
  pkill -f '[c]ircle_draw_node' 2>/dev/null || true
  arm_release_serial
}
trap cleanup EXIT

for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '/joint_states'; then
    break
  fi
  sleep 0.5
done

python3 "${RECORD_PY}" \
  --duration "${DURATION}" \
  --demo "${DEMO_LABEL}" \
  --arm-type a_l1 \
  --out "${OUT_DIR}" &
RECORD_PID=$!

sleep 1.0
cd "${PROJECT_DIR}/scripts"
python3 circle_draw_node.py --mode "${MODE}" --max-joint-velocity "${MAX_VEL}" &
DEMO_PID=$!

sleep "${DURATION}"
kill -INT "${DEMO_PID}" 2>/dev/null || true
wait "${DEMO_PID}" 2>/dev/null || true
wait "${RECORD_PID}" || true

# 补充项目元数据
python3 - <<PY
import json
from pathlib import Path
p = Path("${OUT_DIR}") / "run_meta.json"
meta = json.loads(p.read_text())
meta.update({
    "project": "circle-draw",
    "mode": "${MODE}",
    "max_joint_velocity": float("${MAX_VEL}"),
    "circle_center": [0.35, 0.0, 0.15],
    "circle_radius_m": 0.08,
    "period_sec": 8.0,
    "publish_rate_hz": 100.0,
})
p.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
PY

if [[ -f "${PLOT_PY}" ]]; then
  python3 "${PLOT_PY}" "${OUT_DIR}" || true
fi

echo ""
echo "========== circle-draw 录制完成 =========="
echo "目录: ${OUT_DIR}"
cat "${OUT_DIR}/summary.txt"
