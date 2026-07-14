#!/usr/bin/env python3
"""根据两次录制 CSV 绘制对比曲线 + 汇总表。"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding='utf-8') as f:
        return list(csv.DictReader(f))


def load_summary(path: Path) -> dict:
    p = path / 'summary.json'
    return json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}


def plot_series(out_dir: Path, clik_dir: Path, wbc_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    clik_err = load_csv(clik_dir / 'task_error.csv')
    wbc_err = load_csv(wbc_dir / 'task_error.csv')
    clik_tele = load_csv(clik_dir / 'teleop_tracking.csv')
    wbc_tele = load_csv(wbc_dir / 'teleop_tracking.csv')

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    for rows, label, color in [
        (clik_err, 'CLIK', '#2563eb'),
        (wbc_err, 'WBC+PI', '#dc2626'),
    ]:
        if not rows:
            continue
        t = np.array([float(r['t_sec']) for r in rows])
        wpos = np.array([float(r['world_pos_err_m']) * 1000 for r in rows])
        wori = np.array([float(r['world_orient_err_rad']) * 1000 for r in rows])
        axes[0].plot(t, wpos, label=label, color=color, alpha=0.85, linewidth=0.8)
        axes[1].plot(t, wori, label=label, color=color, alpha=0.85, linewidth=0.8)
        if 'solve_time_us' in rows[0]:
            st = np.array([float(r.get('solve_time_us', 0)) for r in rows])
            axes[2].plot(t, st, label=label, color=color, alpha=0.85, linewidth=0.8)

    for rows, label, color in [
        (clik_tele, 'CLIK 主从', '#2563eb'),
        (wbc_tele, 'WBC 主从', '#dc2626'),
    ]:
        if not rows:
            continue
        t = np.array([float(r['t_sec']) for r in rows])
        ms = np.array([float(r['master_slave_pos_err_m']) * 1000 for r in rows])
        ax_ms = axes[0].twinx()
        ax_ms.plot(t, ms, '--', color=color, alpha=0.45, linewidth=0.7)

    axes[0].set_ylabel('World EE pos err (mm)')
    axes[0].set_title('CLIK vs WBC (teleop_compare_mode_b.yaml)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('World orient err (mrad)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('Solve time (µs/cycle)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / 'compare_timeseries.png', dpi=150)
    plt.close(fig)

    # RMS bar chart
    sc = load_summary(clik_dir)
    sw = load_summary(wbc_dir)
    if sc and sw:
        fig, ax = plt.subplots(figsize=(8, 4))
        metrics = ['world_pos_mm', 'master_slave_pos_mm', 'solve_time_us']
        labels = ['World EE pos\nRMS (mm)', 'Master-slave pos\nRMS (mm)', 'Solve time\nmean (us)']

        def _val(s: dict, m: str) -> float:
            key = m
            if m == 'world_pos_mm' and m not in s and 'task_pos_mm' in s:
                key = 'task_pos_mm'
            if key not in s:
                return 0.0
            return s[key]['mean'] if m == 'solve_time_us' else s[key]['rms']

        x = np.arange(len(metrics))
        w = 0.35
        clik_vals = [_val(sc, m) for m in metrics]
        wbc_vals = [_val(sw, m) for m in metrics]
        ax.bar(x - w / 2, clik_vals, w, label='CLIK', color='#2563eb')
        ax.bar(x + w / 2, wbc_vals, w, label='WBC+PI', color='#dc2626')
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_title('Summary metrics')
        ax.legend()
        ax.grid(True, axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / 'compare_bars.png', dpi=150)
        plt.close(fig)

        lines = [
            '# 对比结果',
            '',
            '| 指标 | CLIK | WBC+积分 | 更优 |',
            '|------|------|----------|------|',
        ]
        def _get(s: dict, m: str) -> dict:
            if m in s:
                return s[m]
            if m == 'world_pos_mm' and 'task_pos_mm' in s:
                return s['task_pos_mm']
            return {'rms': 0.0, 'max': 0.0, 'mean': 0.0, 'p95': 0.0}

        rows_def = [
            ('World EE pos RMS (mm)', _get(sc, 'world_pos_mm')['rms'], _get(sw, 'world_pos_mm')['rms']),
            ('World EE pos max (mm)', _get(sc, 'world_pos_mm')['max'], _get(sw, 'world_pos_mm')['max']),
            ('Master-slave pos RMS (mm)', _get(sc, 'master_slave_pos_mm')['rms'], _get(sw, 'master_slave_pos_mm')['rms']),
            ('Solve mean (us)', _get(sc, 'solve_time_us')['mean'], _get(sw, 'solve_time_us')['mean']),
            ('Solve p95 (us)', _get(sc, 'solve_time_us')['p95'], _get(sw, 'solve_time_us')['p95']),
        ]
        for name, cv, wv in rows_def:
            win = 'CLIK' if cv < wv else ('WBC' if wv < cv else 'tie')
            if cv == wv:
                win = 'tie'
            lines.append(f'| {name} | {cv:.3f} | {wv:.3f} | {win} |')
        (out_dir / 'COMPARE_RESULT.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--clik', type=Path, required=True)
    parser.add_argument('--wbc', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plot_series(args.out, args.clik, args.wbc)
    print(f'曲线已保存: {args.out}/compare_timeseries.png')
    print(f'柱状图: {args.out}/compare_bars.png')
    result = args.out / 'COMPARE_RESULT.md'
    if result.exists():
        print(result.read_text(encoding='utf-8'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
