# 运行数据目录（由 `./run_record.sh` / `record_failure_analysis.py` 生成）

每次运行落在 `runs/<时间戳>_…/`，核心文件：

| 文件 | 用途 |
|------|------|
| `aligned.csv` | 对齐主表，优先用来画曲线 / 找失效时刻 |
| `summary.txt` | 当场 RMS / 发散粗判 |
| `events.csv` | 手动标记（录制中按 Enter / `f`） |
| `streams/*.csv` | 各话题原始流 |
| `bag/` | 可选 `ros2 bag`（`--bag`） |
| `run_meta.json` | 元数据 |
| `overview_delta_err.png` | 由 `./run_plot_mode_b.sh` 生成：全程 Δ vs EE 误差 |
| `plots_lon5s/` 等 | 纵向 / 横向 / 峰值 5 s 窗图（`ee_pose_6panel`、`delta_vs_err`、`joints_cmd_fb`） |
| `plot_summary.json` | 绘图窗口时间与简要指标 |

失效判断提示见每次 `summary.txt` 末尾。

## 绘图

录制后请用统一入口（勿再写临时脚本），详见项目根目录 `README.md`「录制后绘图」：

```bash
cd ~/zihan_ws/arm/projects/ee-stabilization-vicon
./run_plot_mode_b.sh                          # 最新 run
./run_plot_mode_b.sh data/runs/<stamp>_…      # 指定 run
TITLE=MyNote ./run_plot_mode_b.sh --latest
```
