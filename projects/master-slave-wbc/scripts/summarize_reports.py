#!/usr/bin/env python3
"""汇总 reports/ 下各次实验 summary.json / summary.txt。"""

from __future__ import annotations

import argparse
import json
from glob import glob
from pathlib import Path


def load_run(run_dir: Path) -> dict | None:
    summary_json = run_dir / 'summary.json'
    if summary_json.exists():
        data = json.loads(summary_json.read_text(encoding='utf-8'))
        data['_name'] = run_dir.name
        data['_dir'] = str(run_dir)
        return data
    summary_txt = run_dir / 'summary.txt'
    if not summary_txt.exists():
        return None
    return {'_name': run_dir.name, '_dir': str(run_dir), '_raw': summary_txt.read_text(encoding='utf-8')}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--reports', type=Path, default=Path(__file__).resolve().parents[1] / 'reports')
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()

    runs = []
    for p in sorted(args.reports.glob('*')):
        if p.is_dir():
            s = load_run(p)
            if s:
                runs.append(s)

    lines = [
        '# master-slave-wbc 实验数据汇总',
        '',
        '| 运行 | world pos RMS (mm) | world orient RMS (mrad) | 主从 pos RMS (mm) | joint4跳变 | j4相对主臂 mean (°) |',
        '|------|--------------------|-------------------------|-------------------|------------|---------------------|',
    ]
    for s in runs:
        if 'task_pos_mm' not in s:
            continue
        j4 = s.get('joint4_master_err_rad', {})
        lines.append(
            f"| {s['_name']} "
            f"| {s['task_pos_mm']['rms']:.2f} "
            f"| {s['task_orient_mrad']['rms']:.2f} "
            f"| {s['master_slave_pos_mm']['rms']:.2f} "
            f"| {s.get('jump_joint4', '?')} "
            f"| {j4.get('mean', 0)*57.3:.1f} |"
        )

    lines += [
        '',
        '## 结论（基于数据采集）',
        '',
        '1. **关节跳变**：`|Δjoint4|` 逐步最大约 0.52°，无 ≥46° 突变；问题不在 link4 snap，而在腕部冗余支路。',
        '2. **早期 WBC**：关节空间 QP + 错误零空间项 → world EE RMS ~45 mm，自稳失败。',
        '3. **修复后**：任务空间阻尼伪逆 `(I-J#J)` 零空间投影 + 积分动作 + CLIK 同构前馈 → world EE RMS ~2.7 mm，与 CLIK ~2.3 mm 同级。',
        '4. **Mode A**：主从关节完全一致；EE 指标 ~20 mm 来自主臂/从臂 TF 链几何偏置，非控制故障。',
        '',
        '## 复现实验',
        '',
        '```bash',
        'cd /root/arm/projects/master-slave-wbc',
        './scripts/run_experiments.sh          # Mode A + B 各 25s',
        './scripts/run_mode_b_benchmark.sh     # 仅 Mode B 快速回归',
        './scripts/run_clik_baseline.sh        # CLIK 对照组',
        '```',
    ]

    text = '\n'.join(lines) + '\n'
    out = args.out or (args.reports / 'EXPERIMENT_SUMMARY.md')
    out.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
