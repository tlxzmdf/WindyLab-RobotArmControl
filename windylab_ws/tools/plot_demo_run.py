#!/usr/bin/env python3
"""根据 record_demo_run.py 输出绘制关节曲线与跟踪误差图。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _read_csv(path: Path) -> tuple[list[float], dict[str, list[float]]]:
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return [], {}
    t = [float(r['t_sec']) for r in rows]
    series: dict[str, list[float]] = {}
    for key in rows[0].keys():
        if key == 't_sec':
            continue
        series[key] = [float(r[key]) for r in rows]
    return t, series


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path, help='run_data/<timestamp>_<demo> 目录')
    args = parser.parse_args()
    run_dir = args.run_dir
    if not run_dir.is_dir():
        raise SystemExit(f'目录不存在: {run_dir}')

    t_state, state = _read_csv(run_dir / 'joint_states.csv')
    t_cmd, cmd = _read_csv(run_dir / 'joint_command.csv')
    t_err, err = _read_csv(run_dir / 'tracking_error.csv')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f'Demo run: {run_dir.name}', fontsize=14)

    for i in range(1, 4):
        j = f'joint{i}'
        axes[0].plot(t_cmd, cmd[f'{j}_pos'], label=f'{j} cmd', alpha=0.8)
        axes[0].plot(t_state, state[f'{j}_pos'], '--', label=f'{j} actual', alpha=0.8)
    axes[0].set_ylabel('position (rad)')
    axes[0].legend(ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Joints 1-3: command vs actual')

    for i in range(1, 4):
        j = f'joint{i}'
        axes[1].plot(t_err, err[f'{j}_err'], label=j)
    axes[1].set_ylabel('error (rad)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Tracking error (cmd - actual)')

    for i in range(1, 4):
        j = f'joint{i}'
        axes[2].plot(t_state, state[f'{j}_vel'], label=j)
    axes[2].set_xlabel('time (s)')
    axes[2].set_ylabel('velocity (rad/s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Actual joint velocity')

    out = run_dir / 'analysis_plot.png'
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'已保存: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
