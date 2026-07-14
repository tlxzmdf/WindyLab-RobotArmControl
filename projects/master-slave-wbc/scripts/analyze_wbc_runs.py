#!/usr/bin/env python3
"""对比多组 WBC 实验 summary.json，输出 Markdown 报告。"""

from __future__ import annotations

import argparse
import json
from glob import glob
from pathlib import Path


def load_summary(run_dir: Path) -> dict | None:
    p = run_dir / 'summary.json'
    if not p.exists():
        return None
    data = json.loads(p.read_text(encoding='utf-8'))
    data['_dir'] = str(run_dir)
    data['_name'] = run_dir.name
    return data


def fmt_stats(d: dict, scale: float = 1.0, unit: str = '') -> str:
    return f"rms={d['rms']*scale:.2f}{unit} max={d['max']*scale:.2f}{unit} p95={d['p95']*scale:.2f}{unit}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch', nargs='+', required=True, help='glob patterns for run dirs')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    dirs: list[Path] = []
    for pattern in args.batch:
        dirs.extend(Path(p) for p in sorted(glob(pattern)))

    summaries = [s for d in dirs if (s := load_summary(d)) is not None]
    if not summaries:
        print('No summaries found')
        return 1

    lines = [
        '# WBC 实验对比',
        '',
        '| 实验 | joint4跳变 | 任务pos RMS (mm) | 任务orient RMS (mrad) | 主从pos RMS (mm) | 主从orient RMS (mrad) | j4跟踪max (°) |',
        '|------|------------|------------------|------------------------|------------------|------------------------|---------------|',
    ]
    for s in summaries:
        name = s['_name'].replace(f"{s['_name'].split('_')[0]}_", '', 1) if '_' in s['_name'] else s['_name']
        lines.append(
            f"| {name} "
            f"| {s['jump_joint4']} "
            f"| {s['task_pos_mm']['rms']:.2f} "
            f"| {s['task_orient_mrad']['rms']:.2f} "
            f"| {s['master_slave_pos_mm']['rms']:.2f} "
            f"| {s['master_slave_orient_mrad']['rms']:.2f} "
            f"| {s['joint4_master_err_rad']['max']*57.3:.1f} |"
        )

    lines += ['', '## 详细', '']
    for s in summaries:
        lines.append(f"### {s['_name']}")
        lines.append(f"- 目录: `{s['_dir']}`")
        lines.append(f"- 跳变: total={s['jump_total']} joint4={s['jump_joint4']} joint6={s['jump_joint6']}")
        lines.append(f"- 任务误差 pos: {fmt_stats(s['task_pos_mm'], unit='mm')}")
        lines.append(f"- 主从跟踪 pos: {fmt_stats(s['master_slave_pos_mm'], unit='mm')}")
        lines.append('')

    # Recommendations
    b_runs = [s for s in summaries if 'mode_b' in s['_name']]
    a_runs = [s for s in summaries if 'mode_a' in s['_name']]
    lines.append('## 诊断')
    if b_runs:
        b = b_runs[0]
        if b['jump_joint4'] > 0:
            lines.append('- Mode B 存在 joint4 腕部冗余支路跳变 → 需腕部连续支路选择 + 子步内刷新任务误差')
        if b['master_slave_pos_mm']['rms'] > 30:
            lines.append('- 主从 EE 跟踪 RMS > 30mm → 积分增益/子步数/目标滤波需调参或加强前馈')
        if b['task_pos_mm']['rms'] > 15:
            lines.append('- 基座系任务误差偏大 → 检查 mount 前馈与 WBC 任务权重')
    if a_runs and b_runs:
        if a_runs[0]['jump_joint4'] == 0 and b_runs[0]['jump_joint4'] > 0:
            lines.append('- Mode A 无跳变、Mode B 有跳变 → 问题在 WBC/IK 支路而非主臂轨迹')

    args.out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(args.out.read_text(encoding='utf-8'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
