#!/usr/bin/env python3
"""真机/仿真录制数据误差分析。

读取 record_demo_run.py / circle-draw record_hw.sh 生成的目录，计算:
  - 关节跟踪误差 (RMS / 峰值 / P95)
  - 末端位置误差 (Pinocchio FK: 指令 vs 实际)
  - 画圆任务: 实际末端 vs 理想圆轨迹
  - 指令关节速度饱和率 (相对 max_velocity)

用法:
  source ~/arm/.pc_arm_env.sh
  python3 analyze_run_data.py ~/arm/run_data/20260626_170654_circle_draw_diff
  python3 analyze_run_data.py ~/arm/run_data --compare
  python3 analyze_run_data.py ~/arm/run_data --compare --report ~/arm/run_data/analysis_report.md
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# pinocchio_ik 路径
_ARM_ROOT = Path(__file__).resolve().parents[2]
_DEMO_DIR = _ARM_ROOT / 'windylab_ws' / 'src' / 'arm-platform' / 'demo'
if str(_DEMO_DIR) not in sys.path:
    sys.path.insert(0, str(_DEMO_DIR))

from pinocchio_ik import PinocchioIK  # noqa: E402

JOINT_NAMES = [f'joint{i}' for i in range(1, 8)]
EE_FRAME = 'link7'


@dataclass
class RunMetrics:
    run_id: str
    demo: str
    arm_type: str
    max_joint_velocity: float
    duration_sec: float
    joint_rms: dict[str, float] = field(default_factory=dict)
    joint_max: dict[str, float] = field(default_factory=dict)
    joint_p95: dict[str, float] = field(default_factory=dict)
    ee_cmd_rms_mm: float = 0.0
    ee_cmd_max_mm: float = 0.0
    ee_path_rms_mm: Optional[float] = None
    ee_path_max_mm: Optional[float] = None
    cmd_saturation_pct: float = 0.0
    states_hz: float = 0.0
    cmd_hz: float = 0.0
    notes: list[str] = field(default_factory=list)


def _read_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _nearest_row(rows: list[dict], t: float) -> Optional[dict]:
    if not rows:
        return None
    return min(rows, key=lambda r: abs(float(r['t_sec']) - t))


def _align(states: list[dict], commands: list[dict]) -> list[tuple[float, np.ndarray, np.ndarray]]:
    out: list[tuple[float, np.ndarray, np.ndarray]] = []
    for s in states:
        c = _nearest_row(commands, float(s['t_sec']))
        if c is None:
            continue
        q_act = np.array([float(s[f'{j}_pos']) for j in JOINT_NAMES])
        q_cmd = np.array([float(c[f'{j}_pos']) for j in JOINT_NAMES])
        out.append((float(s['t_sec']), q_act, q_cmd))
    return out


def _circle_target(t: float, center: np.ndarray, radius: float, period: float) -> np.ndarray:
    w = 2.0 * math.pi / period
    return center + radius * np.array([0.0, math.cos(w * t), math.sin(w * t)])


def _rms(arr: np.ndarray) -> float:
    return float(np.sqrt(np.mean(arr ** 2))) if len(arr) else 0.0


def _p95(arr: np.ndarray) -> float:
    return float(np.percentile(np.abs(arr), 95)) if len(arr) else 0.0


def _estimate_hz(rows: list[dict]) -> float:
    if len(rows) < 2:
        return 0.0
    dt = float(rows[-1]['t_sec']) - float(rows[0]['t_sec'])
    return (len(rows) - 1) / dt if dt > 0 else 0.0


def _cmd_saturation(commands: list[dict], max_vel: float, dt_nom: float = 0.01) -> float:
    """估算相邻指令间所需关节速度超过 max_velocity 的比例。"""
    if len(commands) < 2 or max_vel <= 0:
        return 0.0
    saturated = 0
    total = 0
    prev = None
    for row in commands:
        q = np.array([float(row[f'{j}_pos']) for j in JOINT_NAMES])
        t = float(row['t_sec'])
        if prev is not None:
            dt = max(t - prev[0], 1e-6)
            dq = np.abs(q - prev[1]) / dt
            total += len(JOINT_NAMES)
            saturated += int(np.sum(dq > max_vel * 1.02))
        prev = (t, q)
    return 100.0 * saturated / total if total else 0.0


def analyze_run(run_dir: Path, ik: PinocchioIK, out_plots: bool = True) -> RunMetrics:
    meta_path = run_dir / 'run_meta.json'
    meta = json.loads(meta_path.read_text()) if meta_path.is_file() else {}
    states = _read_csv(run_dir / 'joint_states.csv')
    commands = _read_csv(run_dir / 'joint_command.csv')
    aligned = _align(states, commands)

    run_id = run_dir.name
    demo = meta.get('demo', run_id)
    arm_type = meta.get('arm_type', 'unknown')
    max_vel = float(meta.get('max_joint_velocity', meta.get('max_velocity', 0.2)))
    duration = float(meta.get('duration_sec', 0.0))

    m = RunMetrics(
        run_id=run_id,
        demo=demo,
        arm_type=arm_type,
        max_joint_velocity=max_vel,
        duration_sec=duration,
        states_hz=_estimate_hz(states),
        cmd_hz=_estimate_hz(commands),
        cmd_saturation_pct=_cmd_saturation(commands, max_vel),
    )

    if len(aligned) < 10:
        m.notes.append('样本过少，跳过详细分析')
        return m

    times = np.array([a[0] for a in aligned])
    q_act = np.stack([a[1] for a in aligned])
    q_cmd = np.stack([a[2] for a in aligned])
    joint_err = q_cmd - q_act

    for i, jn in enumerate(JOINT_NAMES):
        e = joint_err[:, i]
        m.joint_rms[jn] = _rms(e)
        m.joint_max[jn] = float(np.max(np.abs(e)))
        m.joint_p95[jn] = _p95(e)

    ee_cmd_err = []
    ee_path_err = []
    center = meta.get('circle_center')
    radius = meta.get('circle_radius_m')
    period = meta.get('period_sec', 8.0)
    is_circle = center is not None and radius is not None

    for t, qa, qc in aligned:
        p_act, _ = ik.forward(qa)
        p_cmd, _ = ik.forward(qc)
        ee_cmd_err.append(np.linalg.norm(p_cmd - p_act))
        if is_circle:
            p_des = _circle_target(t, np.array(center), float(radius), float(period))
            ee_path_err.append(np.linalg.norm(p_des - p_act))

    ee_cmd_err = np.array(ee_cmd_err)
    m.ee_cmd_rms_mm = _rms(ee_cmd_err) * 1000.0
    m.ee_cmd_max_mm = float(np.max(ee_cmd_err)) * 1000.0 if len(ee_cmd_err) else 0.0

    if ee_path_err:
        ee_path_err = np.array(ee_path_err)
        m.ee_path_rms_mm = _rms(ee_path_err) * 1000.0
        m.ee_path_max_mm = float(np.max(ee_path_err)) * 1000.0

    if m.cmd_saturation_pct > 15:
        m.notes.append(f'指令关节速度饱和率 {m.cmd_saturation_pct:.1f}%（>{max_vel} rad/s）')
    if m.cmd_hz > 0 and m.cmd_hz < 80 and 'circle' in demo.lower():
        m.notes.append(f'指令频率偏低 {m.cmd_hz:.0f} Hz，建议 100 Hz')
    if m.ee_path_rms_mm and m.ee_path_rms_mm > 15:
        m.notes.append(f'末端偏离理想轨迹 RMS {m.ee_path_rms_mm:.1f} mm')

    if out_plots:
        path_mm = np.array(ee_path_err) * 1000.0 if len(ee_path_err) else None
        _plot_run(run_dir, times, joint_err, ee_cmd_err * 1000.0, path_mm, m)

    return m


def _plot_run(
    run_dir: Path,
    times: np.ndarray,
    joint_err: np.ndarray,
    ee_cmd_mm: np.ndarray,
    ee_path_mm: Optional[np.ndarray],
    m: RunMetrics,
) -> None:
    nrows = 3 if ee_path_mm is not None else 2
    fig, axes = plt.subplots(nrows, 1, figsize=(12, 3.2 * nrows), sharex=True)
    if nrows == 2:
        axes = list(axes)
    fig.suptitle(f'{m.run_id}\n{m.demo}', fontsize=11)

    for i in range(3):
        axes[0].plot(times, joint_err[:, i], label=JOINT_NAMES[i])
    axes[0].set_ylabel('joint err (rad)')
    axes[0].legend(ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Joint tracking error (cmd - actual)')

    axes[1].plot(times, ee_cmd_mm, color='C3')
    axes[1].set_ylabel('EE err (mm)')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title(f'EE FK error (cmd vs actual)  RMS={m.ee_cmd_rms_mm:.1f} mm')

    if ee_path_mm is not None:
        axes[2].plot(times, ee_path_mm, color='C1')
        axes[2].set_xlabel('time (s)')
        axes[2].set_ylabel('EE err (mm)')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_title(f'EE vs ideal path  RMS={m.ee_path_rms_mm:.1f} mm')
    else:
        axes[1].set_xlabel('time (s)')

    fig.tight_layout()
    fig.savefig(run_dir / 'error_analysis.png', dpi=150)
    plt.close(fig)


def _format_metrics_table(metrics: list[RunMetrics]) -> str:
    lines = [
        '| 运行 | Demo | max_vel | EE RMS (mm) | 路径 RMS (mm) | 饱和率 | joint1 RMS |',
        '|------|------|---------|-------------|---------------|--------|------------|',
    ]
    for m in sorted(metrics, key=lambda x: x.run_id):
        path = f'{m.ee_path_rms_mm:.1f}' if m.ee_path_rms_mm is not None else '—'
        j1 = m.joint_rms.get('joint1', 0.0)
        short = m.run_id.replace('20260626_', '')
        lines.append(
            f'| {short} | {m.demo[:28]} | {m.max_joint_velocity:.2f} | '
            f'{m.ee_cmd_rms_mm:.1f} | {path} | {m.cmd_saturation_pct:.1f}% | {j1:.3f} |'
        )
    return '\n'.join(lines)


def _recommendations(metrics: list[RunMetrics]) -> str:
    lines = [
        '## 误差原因与避免方法',
        '',
        '### 1. 底层限速（最主要）',
        '',
        '`SmoothPositionController` 每 10 ms 将关节位置增量限制在 `±max_velocity × dt`。',
        '若规划器单步所需速度超过 `max_velocity`，真机必然滞后，表现为关节/末端误差累积。',
        '',
        '**避免：**',
        '- 画圆/快速轨迹：`ARM_MAX_VELOCITY=0.35` 启动 launch（与 `circle-draw` 规划一致）',
        '- 规划侧限制单步：`max_joint_step = max_velocity × dt`（`circle_draw_node` 已做）',
        '- 离线检查饱和率：本工具 `cmd_saturation_pct`，目标 < 10%',
        '',
        '### 2. 指令频率与控制器周期不匹配',
        '',
        '`student_arm_node` 控制周期 100 Hz；`move_arm_ik_demo` 仅 50 Hz 且无速度前馈。',
        '',
        '**避免：**',
        '- 用 `circle-draw`（100 Hz）替代原版 `move_arm_ik_demo`',
        '- 模式选 `diff`（在线微分 IK + 速度前馈）',
        '',
        '### 3. 速度前馈未被位置控制器使用',
        '',
        '当前 `SmoothPositionController` **只跟踪位置 setpoint**，`JointState.velocity` 写入 setpoint.dq 但未参与计算。',
        '因此发布 `velocity` 对真机帮助有限，主要仍靠位置追赶。',
        '',
        '**避免（进阶）：**',
        '- 仿真验证可开 `kinematic_mode:=True`（无限速）',
        '- 或扩展控制器使用 `setpoint.dq` 前馈（需改 C++）',
        '',
        '### 4. 位置 IK 帧间跳变',
        '',
        '原版 demo 每帧独立迭代 IK，关节增量偶发超限。',
        '',
        '**避免：**',
        '- 微分 IK + 热启动（`circle-draw --mode diff`）',
        '- 或 `precompute` 离线轨迹查表',
        '',
        '### 5. 录制与启动时序',
        '',
        'Demo 应在收到 `/joint_states` 后再发指令；`rotate_link5` 已等待反馈。',
        '',
        '**避免：**',
        '- 终端 1 launch 稳定 2 s 后再启 Demo',
        '- 使用 `record_hw.sh` / `pc_arm_record_demo.sh` 自动等待话题',
        '',
    ]

    # 数据驱动的对比
    circle_runs = [m for m in metrics if m.ee_path_rms_mm is not None]
    if len(circle_runs) >= 2:
        best = min(circle_runs, key=lambda m: m.ee_path_rms_mm or 1e9)
        worst = max(circle_runs, key=lambda m: m.ee_path_rms_mm or 0)
        lines.extend([
            '### 6. 本次录制对比（数据结论）',
            '',
            f'- **最佳画圆**：`{best.run_id}` — 路径 RMS **{best.ee_path_rms_mm:.1f} mm**',
            f'- **较差画圆**：`{worst.run_id}` — 路径 RMS **{worst.ee_path_rms_mm:.1f} mm**',
            f'- 推荐真机画圆：`circle-draw --mode diff`，`max_velocity=0.35`',
            '',
        ])

    ik_demo = next((m for m in metrics if 'move_arm_ik_demo' in m.demo), None)
    circle_best = next((m for m in metrics if 'circle_draw_diff' in m.run_id and m.max_joint_velocity >= 0.34), None)
    if ik_demo and circle_best and circle_best.ee_path_rms_mm and ik_demo.ee_cmd_rms_mm:
        lines.append(
            f'- 原版 `move_arm_ik_demo` 末端 RMS {ik_demo.ee_cmd_rms_mm:.1f} mm → '
            f'`circle-draw diff` 路径 RMS {circle_best.ee_path_rms_mm:.1f} mm'
        )
        lines.append('')

    return '\n'.join(lines)


def discover_runs(root: Path) -> list[Path]:
    if (root / 'joint_states.csv').is_file():
        return [root]
    runs = []
    for p in sorted(root.iterdir()):
        if p.is_dir() and (p / 'joint_states.csv').is_file():
            runs.append(p)
    return runs


def main() -> int:
    parser = argparse.ArgumentParser(description='分析 run_data 录制误差')
    parser.add_argument('path', type=Path, help='单次运行目录或 run_data 根目录')
    parser.add_argument('--compare', action='store_true', help='分析目录下全部运行并对比')
    parser.add_argument('--report', type=Path, default=None, help='输出 Markdown 报告路径')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()

    runs = discover_runs(args.path) if args.compare or not (args.path / 'joint_states.csv').is_file() else [args.path]
    if not runs:
        print(f'[FAIL] 未找到录制数据: {args.path}', file=sys.stderr)
        return 1

    # 跳过不完整录制
    runs = [r for r in runs if len(_read_csv(r / 'joint_states.csv')) >= 50]
    # 排除已知不完整录制
    skip = {
        '20260626_165852_move_arm_demo',
        '20260626_170535_circle_draw_diff',
        '20260626_170606_circle_draw_precompute',
    }
    runs = [r for r in runs if r.name not in skip]

    ik = PinocchioIK()
    metrics: list[RunMetrics] = []
    for run_dir in runs:
        print(f'分析: {run_dir.name}')
        m = analyze_run(run_dir, ik, out_plots=not args.no_plots)
        metrics.append(m)
        _print_single(m)

    if len(metrics) > 1:
        print('\n' + '=' * 60)
        print('对比表')
        print(_format_metrics_table(metrics))

    report_path = args.report or (args.path / 'analysis_report.md' if args.compare else runs[0] / 'error_report.md')
    report_body = [
        '# 机械臂运行误差分析报告',
        '',
        _format_metrics_table(metrics),
        '',
        _recommendations(metrics),
    ]
    for m in metrics:
        report_body.extend([
            f'## {m.run_id}',
            '',
            f'- Demo: `{m.demo}`',
            f'- max_joint_velocity: {m.max_joint_velocity} rad/s',
            f'- 末端 FK RMS: {m.ee_cmd_rms_mm:.2f} mm',
        ])
        if m.ee_path_rms_mm is not None:
            report_body.append(f'- 理想轨迹 RMS: {m.ee_path_rms_mm:.2f} mm')
        report_body.append(f'- 指令饱和率: {m.cmd_saturation_pct:.1f}%')
        if m.notes:
            report_body.append('- 备注: ' + '; '.join(m.notes))
        report_body.append('')

    report_path.write_text('\n'.join(report_body), encoding='utf-8')
    print(f'\n报告已保存: {report_path}')
    return 0


def _print_single(m: RunMetrics) -> None:
    print(f'  [{m.run_id}] EE FK RMS={m.ee_cmd_rms_mm:.1f} mm', end='')
    if m.ee_path_rms_mm is not None:
        print(f', 路径 RMS={m.ee_path_rms_mm:.1f} mm', end='')
    print(f', 饱和率={m.cmd_saturation_pct:.1f}%')
    top = sorted(m.joint_rms.items(), key=lambda x: -x[1])[:3]
    print(f'  关节 RMS top3: ' + ', '.join(f'{k}={v:.3f}' for k, v in top))


if __name__ == '__main__':
    raise SystemExit(main())
