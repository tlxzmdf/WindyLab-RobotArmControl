# master-slave-wbc 实验数据汇总

| 运行 | world pos RMS (mm) | world orient RMS (mrad) | 主从 pos RMS (mm) | joint4跳变 | j4相对主臂 mean (°) |
|------|--------------------|-------------------------|-------------------|------------|---------------------|
| 20260626_232608_mode_a_mirror | 20.00 | 0.10 | 20.00 | 0 | 0.0 |
| 20260626_232608_mode_b_wbc | 39.58 | 15.24 | 40.66 | 0 | 5.6 |
| 20260626_232925_mode_b_wbc | 45.31 | 15.32 | 45.56 | 0 | 2.9 |
| 20260626_233044_mode_b_wbc | 2.66 | 7.22 | 3.91 | 0 | 26.6 |
| 20260626_233214_mode_b_wbc | 2.76 | 7.62 | 4.01 | 0 | 0.0 |

## 结论（基于数据采集）

1. **关节跳变**：`|Δjoint4|` 逐步最大约 0.52°，无 ≥46° 突变；问题不在 link4 snap，而在腕部冗余支路。
2. **早期 WBC**：关节空间 QP + 错误零空间项 → world EE RMS ~45 mm，自稳失败。
3. **修复后**：任务空间阻尼伪逆 `(I-J#J)` 零空间投影 + 积分动作 + CLIK 同构前馈 → world EE RMS ~2.7 mm，与 CLIK ~2.3 mm 同级。
4. **Mode A**：主从关节完全一致；EE 指标 ~20 mm 来自主臂/从臂 TF 链几何偏置，非控制故障。

## 复现实验

```bash
cd /root/arm/projects/master-slave-wbc
./scripts/run_experiments.sh          # Mode A + B 各 25s
./scripts/run_mode_b_benchmark.sh     # 仅 Mode B 快速回归
./scripts/run_clik_baseline.sh        # CLIK 对照组
```
