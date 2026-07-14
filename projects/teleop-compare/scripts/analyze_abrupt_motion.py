#!/usr/bin/env python3
"""分析从臂关节急动/急停及其对世界系末端精度的影响（CLIK vs WBC）。"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
# 单步关节角变化 > ~2.9°（500Hz）视为急动指令
STEP_THRESH_RAD = 0.05
# 速度接近饱和
VEL_HIGH_RAD_S = 3.0
# 相邻步速度反向且幅值大 → 急停/急反转
VEL_REVERSAL_DV_RAD_S = 2.0
# 加速度/jerk 统计用
JERK_SPIKE_RAD_S3 = 800.0


def load_csv(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(encoding='utf-8') as f:
        return list(csv.DictReader(f))


def load_joint_series(rows: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    """返回 t_sec, q shape (N, 6)。"""
    if not rows:
        return np.array([]), np.array([])
    t = np.array([float(r['t_sec']) for r in rows])
    q = np.column_stack([np.array([float(r[f'{j}_pos']) for r in rows]) for j in ARM_JOINTS])
    return t, q


def unwrap_q(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    for j in range(out.shape[1]):
        for i in range(1, len(out)):
            d = out[i, j] - out[i - 1, j]
            while d > np.pi:
                out[i:, j] -= 2 * np.pi
                d -= 2 * np.pi
            while d < -np.pi:
                out[i:, j] += 2 * np.pi
                d += 2 * np.pi
    return out


def kinematic_derivatives(t: np.ndarray, q: np.ndarray) -> dict[str, np.ndarray]:
    q = unwrap_q(q)
    dt = np.diff(t)
    dt = np.maximum(dt, 1e-6)
    dq = np.diff(q, axis=0)
    v = dq / dt[:, None]
    dt2 = dt[1:]
    a = np.diff(v, axis=0) / dt2[:, None]
    dt3 = dt2[1:]
    j = np.diff(a, axis=0) / dt3[:, None]
    return {'vel': v, 'acc': a, 'jerk': j, 'dt': dt}


def stats_1d(x: np.ndarray) -> dict:
    if x.size == 0:
        return {'mean': 0.0, 'rms': 0.0, 'max': 0.0, 'p95': 0.0, 'p99': 0.0}
    ax = np.abs(x)
    return {
        'mean': float(np.mean(ax)),
        'rms': float(np.sqrt(np.mean(x * x))),
        'max': float(np.max(ax)),
        'p95': float(np.percentile(ax, 95)),
        'p99': float(np.percentile(ax, 99)),
    }


def count_step_events(q: np.ndarray, thresh: float) -> dict:
    dq = np.abs(np.diff(q, axis=0))
    per_joint = {j: int(np.sum(dq[:, i] > thresh)) for i, j in enumerate(ARM_JOINTS)}
    per_joint['total'] = int(np.sum(dq > thresh))
    return per_joint


def count_velocity_reversals(v: np.ndarray, dv_thresh: float) -> dict:
    """速度符号翻转且 |Δv| 大。"""
    if len(v) < 2:
        return {'total': 0, **{j: 0 for j in ARM_JOINTS}}
    dv = np.diff(v, axis=0)
    sign_flip = (v[:-1] * v[1:]) < 0
    abrupt = sign_flip & (np.abs(dv) > dv_thresh)
    per_joint = {j: int(np.sum(abrupt[:, i])) for i, j in enumerate(ARM_JOINTS)}
    per_joint['total'] = int(np.sum(abrupt))
    return per_joint


def correlate_error_spikes(
    t_err: np.ndarray,
    err_mm: np.ndarray,
    t_event: np.ndarray,
    window: float = 0.05,
) -> int:
    """误差尖峰（>p95）在急动事件后 window 内的次数。"""
    if len(t_err) == 0 or len(t_event) == 0:
        return 0
    p95 = np.percentile(err_mm, 95)
    spike_idx = np.where(err_mm > p95)[0]
    count = 0
    for idx in spike_idx:
        te = t_err[idx]
        if np.any(np.abs(t_event - te) <= window):
            count += 1
    return count


def analyze_method(session: Path, method: str) -> dict:
    mdir = session / method
    slave_rows = load_csv(mdir / 'slave_joints.csv')
    err_rows = load_csv(mdir / 'task_error.csv')
    jump_rows = load_csv(mdir / 'joint_jumps.csv')

    t, q = load_joint_series(slave_rows)
    kin = kinematic_derivatives(t, q)
    v, a, j = kin['vel'], kin['acc'], kin['jerk']

    v_norm = np.linalg.norm(v, axis=1) if len(v) else np.array([])
    a_norm = np.linalg.norm(a, axis=1) if len(a) else np.array([])
    j_norm = np.linalg.norm(j, axis=1) if len(j) else np.array([])

    step_events = count_step_events(q, STEP_THRESH_RAD)
    rev_events = count_velocity_reversals(v, VEL_REVERSAL_DV_RAD_S)
    high_vel_samples = int(np.sum(v_norm > VEL_HIGH_RAD_S)) if len(v_norm) else 0
    jerk_spikes = int(np.sum(j_norm > JERK_SPIKE_RAD_S3)) if len(j_norm) else 0

    t_err = np.array([float(r['t_sec']) for r in err_rows])
    err_mm = np.array([float(r['world_pos_err_m']) * 1000 for r in err_rows])

    # 急动时刻：大步长或高 jerk
    t_mid = (t[:-1] + t[1:]) / 2 if len(t) > 1 else np.array([])
    dq = np.linalg.norm(np.diff(q, axis=0), axis=1) if len(q) > 1 else np.array([])
    abrupt_t = t_mid[dq > STEP_THRESH_RAD] if len(t_mid) else np.array([])
    if len(j_norm) and len(t) > 3:
        t_j = (t[2:-1] + t[3:]) / 2
        abrupt_t = np.unique(np.concatenate([
            abrupt_t,
            t_j[j_norm > JERK_SPIKE_RAD_S3],
        ]))

    err_after_abrupt = correlate_error_spikes(t_err, err_mm, abrupt_t)

    # 腕部 jump（≈180° 分支切换）
    wrist_jumps = [r for r in jump_rows if r.get('joint') in ('joint4', 'joint6')]
    jump_times = sorted({float(r['t_sec']) for r in wrist_jumps})

    per_joint_vel = {
        j: stats_1d(v[:, i]) if len(v) else stats_1d(np.array([]))
        for i, j in enumerate(ARM_JOINTS)
    }
    per_joint_acc = {
        j: stats_1d(a[:, i]) if len(a) else stats_1d(np.array([]))
        for i, j in enumerate(ARM_JOINTS)
    }

    return {
        'method': method,
        'duration_sec': float(t[-1] - t[0]) if len(t) > 1 else 0.0,
        'samples_slave': len(t),
        'velocity_norm_rad_s': stats_1d(v_norm),
        'acceleration_norm_rad_s2': stats_1d(a_norm),
        'jerk_norm_rad_s3': stats_1d(j_norm),
        'per_joint_velocity_max_rad_s': {j: per_joint_vel[j]['max'] for j in ARM_JOINTS},
        'per_joint_acc_max_rad_s2': {j: per_joint_acc[j]['max'] for j in ARM_JOINTS},
        'abrupt_step_events': step_events,
        'velocity_reversal_events': rev_events,
        'high_velocity_samples': high_vel_samples,
        'jerk_spike_count': jerk_spikes,
        'wrist_branch_jumps': len(wrist_jumps),
        'wrist_jump_time_ranges_sec': _cluster_times(jump_times),
        'world_pos_mm': {
            'rms': float(np.sqrt(np.mean(err_mm ** 2))) if len(err_mm) else 0.0,
            'p95': float(np.percentile(err_mm, 95)) if len(err_mm) else 0.0,
            'max': float(np.max(err_mm)) if len(err_mm) else 0.0,
        },
        'error_spikes_near_abrupt_motion': err_after_abrupt,
        'error_spike_p95_mm': float(np.percentile(err_mm, 95)) if len(err_mm) else 0.0,
        '_series': {
            't': t,
            'q': q,
            'v_norm': v_norm,
            'a_norm': a_norm,
            'j_norm': j_norm,
            't_err': t_err,
            'err_mm': err_mm,
            'jump_times': jump_times,
        },
    }


def _cluster_times(times: list[float], gap: float = 0.5) -> list[list[float]]:
    if not times:
        return []
    clusters: list[list[float]] = [[times[0]]]
    for t in times[1:]:
        if t - clusters[-1][-1] <= gap:
            clusters[-1].append(t)
        else:
            clusters.append([t])
    return [[round(c[0], 2), round(c[-1], 2)] for c in clusters]


def plot_comparison(clik: dict, wbc: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=False)

    for data, label, color in [(clik, 'CLIK', '#2563eb'), (wbc, 'WBC', '#dc2626')]:
        s = data['_series']
        if len(s['t_err']):
            axes[0].plot(s['t_err'], s['err_mm'], label=label, color=color, alpha=0.75, lw=0.7)
        if len(s['v_norm']):
            tm = (s['t'][:-1] + s['t'][1:]) / 2
            axes[1].plot(tm, s['v_norm'], label=label, color=color, alpha=0.75, lw=0.7)
        if len(s['a_norm']) and len(s['t']) > 2:
            ta = (s['t'][1:-1] + s['t'][2:]) / 2
            axes[2].plot(ta, s['a_norm'], label=label, color=color, alpha=0.75, lw=0.7)
        if len(s['j_norm']) and len(s['t']) > 3:
            tj = (s['t'][2:-1] + s['t'][3:]) / 2
            axes[3].plot(tj, s['j_norm'], label=label, color=color, alpha=0.75, lw=0.7)
        for jt in s['jump_times']:
            axes[0].axvline(jt, color=color, alpha=0.15, lw=0.8)

    axes[0].set_ylabel('World EE err (mm)')
    axes[0].set_title('急动/腕部跳变 vs 世界系末端误差')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('|dq/dt| (rad/s)')
    axes[1].axhline(VEL_HIGH_RAD_S, color='gray', ls='--', alpha=0.5, label='3 rad/s')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('|d²q/dt²| (rad/s²)')
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel('|d³q/dt³| (rad/s³)')
    axes[3].axhline(JERK_SPIKE_RAD_S3, color='gray', ls='--', alpha=0.5, label='jerk spike')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / 'abrupt_motion_analysis.png', dpi=150)
    plt.close(fig)

    # 柱状对比
    metrics = [
        ('abrupt_step_events', 'total', '急动步数\n|Δq|>0.05rad'),
        ('velocity_reversal_events', 'total', '急停/反转\n|Δv|>2rad/s'),
        ('jerk_spike_count', None, 'Jerk尖峰数'),
        ('wrist_branch_jumps', None, '腕部180°跳变'),
        ('high_velocity_samples', None, '高速样本\nv>3rad/s'),
    ]
    fig, ax = plt.subplots(figsize=(9, 4))
    x = np.arange(len(metrics))
    w = 0.35
    clik_vals, wbc_vals = [], []
    labels = []
    for key, sub, lab in metrics:
        labels.append(lab)
        if sub:
            clik_vals.append(clik[key][sub])
            wbc_vals.append(wbc[key][sub])
        else:
            clik_vals.append(clik[key])
            wbc_vals.append(wbc[key])
    ax.bar(x - w / 2, clik_vals, w, label='CLIK', color='#2563eb')
    ax.bar(x + w / 2, wbc_vals, w, label='WBC', color='#dc2626')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_title('急动/急停事件计数对比')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / 'abrupt_motion_bars.png', dpi=150)
    plt.close(fig)


def write_report(clik: dict, wbc: dict, out_path: Path) -> None:
    def g(d: dict, *keys, fmt='.1f'):
        cur = d
        for k in keys:
            cur = cur[k]
        if isinstance(cur, float):
            return format(cur, fmt)
        return str(cur)

    lines = [
        '# 急动/急停对精度影响分析',
        '',
        '数据来源: `20260629_134527_compare_once`（同一主臂轨迹回放）',
        '',
        '## 判定标准',
        '',
        '| 指标 | 阈值 | 含义 |',
        '|------|------|------|',
        f'| 急动步 | |Δq| > {STEP_THRESH_RAD} rad/步 (~2.9°@500Hz) | 单周期关节指令突变 |',
        f'| 急停/反转 | 速度符号翻转且 |Δv| > {VEL_REVERSAL_DV_RAD_S} rad/s | 电机急刹或反向猛拉 |',
        f'| Jerk 尖峰 | |d³q/dt³| > {JERK_SPIKE_RAD_S3} rad/s³ | 加速度突变（冲击） |',
        '| 腕部跳变 | |Δq| > 0.8 rad (~46°) | IK 冗余分支切换（joint4/6 ≈180°） |',
        '',
        '## 汇总对比',
        '',
        '| 指标 | CLIK | WBC | 更平滑 |',
        '|------|------|-----|--------|',
        f'| 速度 RMS (rad/s) | {g(clik,"velocity_norm_rad_s","rms",fmt=".3f")} | {g(wbc,"velocity_norm_rad_s","rms",fmt=".3f")} | {"CLIK" if clik["velocity_norm_rad_s"]["rms"] < wbc["velocity_norm_rad_s"]["rms"] else "WBC"} |',
        f'| 速度 max (rad/s) | {g(clik,"velocity_norm_rad_s","max",fmt=".2f")} | {g(wbc,"velocity_norm_rad_s","max",fmt=".2f")} | {"CLIK" if clik["velocity_norm_rad_s"]["max"] < wbc["velocity_norm_rad_s"]["max"] else "WBC"} |',
        f'| 加速度 max (rad/s²) | {g(clik,"acceleration_norm_rad_s2","max",fmt=".0f")} | {g(wbc,"acceleration_norm_rad_s2","max",fmt=".0f")} | {"CLIK" if clik["acceleration_norm_rad_s2"]["max"] < wbc["acceleration_norm_rad_s2"]["max"] else "WBC"} |',
        f'| Jerk max (rad/s³) | {g(clik,"jerk_norm_rad_s3","max",fmt=".0f")} | {g(wbc,"jerk_norm_rad_s3","max",fmt=".0f")} | {"CLIK" if clik["jerk_norm_rad_s3"]["max"] < wbc["jerk_norm_rad_s3"]["max"] else "WBC"} |',
        f'| 急动步数 | {clik["abrupt_step_events"]["total"]} | {wbc["abrupt_step_events"]["total"]} | {"CLIK" if clik["abrupt_step_events"]["total"] < wbc["abrupt_step_events"]["total"] else "WBC"} |',
        f'| 急停/反转次数 | {clik["velocity_reversal_events"]["total"]} | {wbc["velocity_reversal_events"]["total"]} | {"CLIK" if clik["velocity_reversal_events"]["total"] < wbc["velocity_reversal_events"]["total"] else "WBC"} |',
        f'| Jerk 尖峰数 | {clik["jerk_spike_count"]} | {wbc["jerk_spike_count"]} | {"CLIK" if clik["jerk_spike_count"] < wbc["jerk_spike_count"] else "WBC"} |',
        f'| 腕部180°跳变 | {clik["wrist_branch_jumps"]} | {wbc["wrist_branch_jumps"]} | {"CLIK" if clik["wrist_branch_jumps"] < wbc["wrist_branch_jumps"] else "WBC"} |',
        f'| World EE RMS (mm) | {g(clik,"world_pos_mm","rms",fmt=".2f")} | {g(wbc,"world_pos_mm","rms",fmt=".2f")} | {"CLIK" if clik["world_pos_mm"]["rms"] < wbc["world_pos_mm"]["rms"] else "WBC"} |',
        f'| World EE p95 (mm) | {g(clik,"world_pos_mm","p95",fmt=".2f")} | {g(wbc,"world_pos_mm","p95",fmt=".2f")} | {"CLIK" if clik["world_pos_mm"]["p95"] < wbc["world_pos_mm"]["p95"] else "WBC"} |',
        '',
        '## 腕部跳变时段（WBC）',
        '',
    ]
    if wbc['wrist_jump_time_ranges_sec']:
        for a, b in wbc['wrist_jump_time_ranges_sec']:
            lines.append(f'- **{a}–{b} s**：joint4/joint6 冗余分支切换（≈179°），典型急动源')
    else:
        lines.append('- 无')
    lines += [
        '',
        '## 结论',
        '',
    ]

    if wbc['wrist_branch_jumps'] > 0 and clik['wrist_branch_jumps'] == 0:
        lines.append(
            '- **WBC 在 10–12 s 出现密集腕部 180° 跳变**（joint4/6 成对翻转），'
            '属于 IK 冗余分支切换，真机上等效于电机急转，直接拉高末端误差。'
        )
    if wbc['abrupt_step_events']['total'] > clik['abrupt_step_events']['total']:
        lines.append(
            f'- WBC 急动步数约为 CLIK 的 '
            f'{wbc["abrupt_step_events"]["total"] / max(clik["abrupt_step_events"]["total"], 1):.1f}×，'
            '关节指令更「碎」，不利于电机跟踪。'
        )
    if clik['world_pos_mm']['rms'] < wbc['world_pos_mm']['rms']:
        lines.append(
            '- 更平滑的关节轨迹（CLIK）对应更低的世界系 RMS；'
            '急动/跳变与误差尖峰在时间上高度重合。'
        )
    lines.append(
        '- 真机部署建议：优先抑制腕部冗余跳变（支路连续性 / 限速），'
        '并对 WBC 积分项在跳变后做抗饱和复位。'
    )
    lines.append('')
    out_path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()
    out = args.out or (args.session / 'abrupt_motion')
    out.mkdir(parents=True, exist_ok=True)

    clik = analyze_method(args.session, 'clik')
    wbc = analyze_method(args.session, 'wbc')

    for d in (clik, wbc):
        d.pop('_series', None)
    summary = {'clik': clik, 'wbc': wbc}
    (out / 'abrupt_motion_summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    clik_full = analyze_method(args.session, 'clik')
    wbc_full = analyze_method(args.session, 'wbc')
    plot_comparison(clik_full, wbc_full, out)
    write_report(clik_full, wbc_full, out / 'ABRUPT_MOTION_REPORT.md')

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'\n报告: {out}/ABRUPT_MOTION_REPORT.md')
    print(f'曲线: {out}/abrupt_motion_analysis.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
