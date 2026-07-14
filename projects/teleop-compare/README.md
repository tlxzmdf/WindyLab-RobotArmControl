# teleop-compare — CLIK vs WBC 公平对比

## 1. 统一参数

详见 [COMPARE_PARAMS.md](COMPARE_PARAMS.md)。

对比实验请使用 **`teleop_compare_mode_b.yaml`**（两项目均已同步）。

## 2. 真机对比（无 RViz）

### 一次手拖 · 双方案自动对比（推荐）

**只需手拖一次**，同一主臂轨迹自动回放并分别录制 CLIK / WBC：

```bash
cd /root/arm/projects/teleop-compare
PORT_NAME=/dev/ttyUSB0 DURATION=15 ./run_hw_compare_once.sh
```

流程：
1. 真机手拖 15s → 保存 `master/master_joints.csv`
2. 回放轨迹 + CLIK 从臂 → 录制
3. 回放轨迹 + WBC 从臂 → 录制
4. 自动生成 `plots/compare_*.png` 与 `abrupt_motion/` 急动/急停分析

输出目录：`reports/<时间戳>_compare_once/`

急动分析输出：
- `abrupt_motion/ABRUPT_MOTION_REPORT.md` — 腕部跳变、急动步、精度关联
- `abrupt_motion/abrupt_motion_analysis.png` — 误差与速度/加速度/jerk 时序

### 分开录制（需手拖两次）

**CLIK：**

```bash
cd /root/arm/projects/teleop-compare
METHOD=clik PORT_NAME=/dev/ttyUSB0 DURATION=15 ./run_hw_record.sh
```

**WBC：**

```bash
METHOD=wbc PORT_NAME=/dev/ttyUSB0 DURATION=15 ./run_hw_record.sh
```

数据目录：`reports/<时间戳>_clik/` 与 `reports/<时间戳>_wbc/`。

**可选：launch 已手动启动时，仅录制**

```bash
source /root/arm/windylab_ws/install/setup.bash
python3 /root/arm/projects/teleop-compare/scripts/record_compare_run.py \
  --method clik --duration 15 --out /root/arm/projects/teleop-compare/reports/my_clik
```

## 3. 生成对比曲线

```bash
python3 /root/arm/projects/teleop-compare/scripts/plot_compare.py \
  --clik /root/arm/projects/teleop-compare/reports/<时间戳>_clik \
  --wbc  /root/arm/projects/teleop-compare/reports/<时间戳>_wbc \
  --out  /root/arm/projects/teleop-compare/reports/<时间戳>_plots
```

输出：
- `compare_timeseries.png` — 世界系误差 / 姿态误差 / 求解耗时曲线
- `compare_bars.png` — RMS 柱状对比
- `COMPARE_RESULT.md` — 数值表

## 4. 仿真一键对比（无 RViz）

```bash
cd /root/arm/projects/teleop-compare
DURATION=15 ./run_sim_compare.sh
```

## 5. 求解速度微基准

```bash
python3 scripts/benchmark_solvers.py \
  --out reports/solver_benchmark.json
```

运行时求解耗时见 `task_error.csv` 的 `solve_time_us` 列（每控制周期 CLIK/WBC 子步总耗时）。

## 6. 急动/急停分析（已挂载到一键对比）

对比完成后自动运行；也可对已有会话单独分析：

```bash
python3 scripts/analyze_abrupt_motion.py \
  --session reports/<时间戳>_compare_once
```

输出：`abrupt_motion/ABRUPT_MOTION_REPORT.md`、`abrupt_motion_analysis.png`、`abrupt_motion_bars.png`
