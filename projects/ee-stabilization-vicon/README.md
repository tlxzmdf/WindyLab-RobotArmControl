# 机械臂末端位姿稳定 · Vicon 相对扰动版

本目录由 `projects/ee-stabilization` **复制**而来，仅在本目录改写。  
**不会修改** `projects/ee-stabilization` 或共享 ROS 包源码；控制仍调用 `windylab_ws` 中的 `arm_ee_stabilization_*`。

## 原理

- Vicon（默认 `/vrpn/pregme/pose`）提供的是**飞机**绝对位姿，坐标零点**不是**机载端。
- 在 \(t_0\) 冻结飞机位姿 \(T_{\mathrm{plane}}(t_0)\)，此后计算相对运动  
  \[
  \Delta(t)=T_{\mathrm{plane}}(t_0)^{-1}\,T_{\mathrm{plane}}(t)
  \]
  作为**机载端扰动**。
- 稳定器把末端锁定在启动时的**世界系**位姿；飞机相对 \(t_0\) 怎么动，机载端就按 \(\Delta\) 扰动，末端保持不动。

桥接节点：`scripts/vicon_relative_bridge.py`

| 输出 | 用途 |
|------|------|
| TF `world` → `base_link` | 真机 `base_source:=tf` |
| `/mount_disturbance/pose` `[x,y,z,r,p,y]` | 仿真 `base_source:=external` |
| `/vicon_relative/delta` | 调试 |
| 服务 `/vicon_relative_bridge/latch_t0` | 重新冻结 \(t_0\) |

## 目录

```text
ee-stabilization-vicon/
├── README.md
├── run_hw.sh / run_sim.sh / run_record.sh / run_plot_mode_b.sh
├── data/runs/          # 失效分析录制输出
├── launch/
│   ├── stabilization_vicon_hw.launch.py
│   └── stabilization_vicon_sim.launch.py
└── scripts/
    ├── vicon_relative_bridge.py
    ├── record_failure_analysis.py
    └── plot_mode_b_run.py   # 录制后统一绘图（勿再写临时脚本）
```

## 前置

1. 已编译 `~/zihan_ws/vicon_perception`（`install/setup.bash` 存在）。
2. **真机推荐直接 `./run_hw.sh`**：默认 `START_VRPN=true`，在抢串口之后由本项目自行拉起 VRPN。  
   不要依赖机载 `robot.service` 里的 VRPN——`claim_arm_serial` 为释放 `/dev/ttyTHS3` 常会停掉 `robot.service` / `robot.launch`，把系统 VRPN 一并带走；若此时 `START_VRPN=false`，bridge 收不到 `/vrpn/pregme/pose`，**不会发 `world→base_link`**，控制端 `mount` 恒为 0（晃机无反应）。
3. 飞机在 \(t_0\) 窗口内尽量静止（默认 latch 2 s）。启动日志应出现 `Latched t0 plane pose`；若反复 `No messages on /vrpn/...` 说明 VRPN 未通。

可选自检：

```bash
ros2 topic echo /vrpn/pregme/pose --once
ros2 run tf2_ros tf2_echo world base_link
```

## 真机

```bash
cd ~/zihan_ws/arm/projects/ee-stabilization-vicon

# 默认会：claim 串口 → 启动 vicon_perception VRPN → home → bridge latch → 控制
./run_hw.sh A
./run_hw.sh C
./run_hw.sh E   # Mode B + CLIK 连续参考 / 零空间贴旧解（低滞后防抖）

# 仅当你已在 claim 之后单独开好 VRPN 时才关自动启动：
# START_VRPN=false ./run_hw.sh C
```

常用环境变量：

| 变量 | 默认 | 说明 |
|------|------|------|
| `POSE_TOPIC` | `/vrpn/pregme/pose` | 飞机刚体话题 |
| `LATCH_DELAY` | `2.0` | 首帧后多久冻结 \(t_0\) |
| `ARM_MAX_VELOCITY` | `0.25` | 真机限速 |
| `START_VRPN` | **`true`** | 是否自动 `vrpn_client.launch`（**推荐保持 true**） |
| `VICON_WS` | `~/zihan_ws/vicon_perception/src` | VRPN 工作空间（需含 `install/setup.bash`） |
| `BASE_SOURCE` | `tf` | 设为 `simulated` 可退回原球扰动（不启 bridge） |
| `HOME_BEFORE_STABILIZE` | `true` | 稳定前先回 `q_home` |
| `HOME_DURATION` / `HOME_SETTLE` | `6.0` / `0.8` | 回零插值时长 / 到位保持 |
| `SKIP_HOME` | `0` | 设 `1` 跳过回 home |

回 home 目标（非全零）：`[0, 0.35, -0.55, 0, 0.45, 0]`（+ joint7=`0`），见 `scripts/move_to_home.py`。

重新冻结 \(t_0\)：

```bash
ros2 service call /vicon_relative_bridge/latch_t0 std_srvs/srv/Trigger {}
```

## 失效分析数据录制

另开终端，在 **`./run_hw.sh` 已在跑** 时采集全量数据（误差、Δ、Vicon、指令/反馈、TF；可选 bag）：

```bash
cd ~/zihan_ws/arm/projects/ee-stabilization-vicon
./run_record.sh --duration 60 --mode C --note shake
# 持续采集直到手动停：--duration 0（每 30 s 自动落盘）
# 需要原始 bag 时加 --bag
# 录制中按 Enter 或 f 标记失效时刻；Ctrl+C 提前结束并落盘
```

输出：`data/runs/<时间戳>_…/`（优先看 `aligned.csv` + `summary.txt`）。字段说明见 `scripts/record_failure_analysis.py` 头部注释与 `data/README.md`。

录制前请确认 `summary` / 日志里 **有** `delta_pos_norm` / TF，且 bridge 已 `Latched`；否则数据只能反映“无扰动输入”。

### 录制后绘图（统一入口）

**不要每次手写临时绘图代码。** 用 `./run_plot_mode_b.sh` / `scripts/plot_mode_b_run.py`：从 `aligned.csv` 均匀时间网格插值画连续曲线，并自动切纵向 / 横向 / 峰值 5 s 窗。

```bash
cd ~/zihan_ws/arm/projects/ee-stabilization-vicon

# 最新一次 recording（默认）
./run_plot_mode_b.sh

# 指定 run（相对或绝对路径，或 runs/ 下目录名）
./run_plot_mode_b.sh data/runs/20260717_185428_B_outerfix_1min

# 标题前缀（图注用）
TITLE=OuterFix ./run_plot_mode_b.sh --latest

# 等价 Python 调用
python3 scripts/plot_mode_b_run.py --latest
python3 scripts/plot_mode_b_run.py --run 20260717_185428_B_outerfix_1min --title-prefix OuterFix
python3 scripts/plot_mode_b_run.py --latest --window-sec 5 --no-joints
```

写出到该 run 目录：

| 产物 | 说明 |
|------|------|
| `overview_delta_err.png` | 全程 Δ vs EE 误差 |
| `plots_lon5s/` | 纵向扰动最强 5 s：`ee_pose_6panel`、`delta_vs_err`、`joints_cmd_fb` |
| `plots_lat5s/` | 横向扰动最强 5 s（同上） |
| `plots_peak5s/` | 误差峰值附近 5 s（同上） |
| `plot_summary.json` | 窗口时间与简要指标 |

常用参数：`--run` / `--latest`、`--window-sec`（默认 5）、`--title-prefix`、`--no-joints`、`--dt`（强制重采样步长，0=自动）。

## Mode C 自动调参

增益只在 `ee_stabilization` **启动时**加载，因此每轮 = 写 overlay → 重启 Mode C → 录制 → 打分（越低越好）。真机需人晃机，脚本负责启停与选参。

```bash
cd ~/zihan_ws/arm/projects/ee-stabilization-vicon

# 给最近几轮已有数据打分（无需开臂）
./run_tune_mode_c.sh score --all-recent 5

# 交互自动循环（推荐）：每轮晃机→放回原位≥15s→Enter
./run_tune_mode_c.sh auto --session my_tune --max-trials 6 --strategy coordinate

# 只写一组 overlay，手动启动
./run_tune_mode_c.sh apply --kp-pos 700 --osc-lambda 0.04 --lpf-alpha 0.55
PARAMS_OVERLAY=/path/to/overlay.yaml ./run_hw.sh C
```

会话产物：`data/tune/<session>/trials.jsonl`、`best_overlay.yaml`。打分看重大扰动误差、回正静差（Δ≈0）、e/Δ、抖动与力矩饱和。

## MIT 阻抗增益（student_arm）

真机 `student_arm`（`mit_stabilization`）默认阻抗增益见  
`windylab_ws/src/arm-platform/config/stabilization_hw_student_arm.yaml`。

| 关节 | stock | 2026-07-17 prox tune | **Phase 1 现行** |
|------|-------|----------------------|------------------|
| j1–j3 `p_gain` / `d_gain` | 30 / 1.0 | 60 / 1.5 | **60 / 2.2** |
| j4–j6 | 5 / 0.1 | 5 / 0.1 | **2.5 / 0.45** |
| j7 | 1 / 0.1 | 1 / 0.1 | 1 / 0.1 |

近端 60/1.5 依据：`data/tune_mit_gains/prox_j123_all3/`。Phase 1 在近端略增阻尼、腕部降刚度升阻尼，配合 Mode B 软限速/腕部死区抑锯齿；说明见 `data/mode_b_phase1/CHANGELOG.md`。

```bash
# Phase 0 诊断（对已有 aligned.csv）
python3 scripts/analyze_mode_b_phase0.py --run data/runs/<stamp>_B_...

# 复测三任务 MIT 轨迹
./run_hw_mit_traj_boot.sh

# 再扫近端增益（每组三种运动 + 回零）
./run_tune_mit_gains.sh auto --session my_tune --confirm-hw --task all --strategy grid --max-trials 16
```
## 仿真

```bash
# 需有 /vrpn/pregme/pose（真机或 START_VRPN=true）
./run_sim.sh
# START_VRPN=true ./run_sim.sh
```

## 与原版 ee-stabilization 的关系

| | `ee-stabilization` | `ee-stabilization-vicon`（本目录） |
|--|--------------------|-----------------------------------|
| 扰动来源 | 球内随机 / 可选 TF 绝对位姿 | 飞机相对 \(t_0\) 的 \(\Delta\) |
| 改动范围 | 基线工程 | **仅本目录**脚本与 launch |
| 控制包 | 同一套 `arm_ee_stabilization_*` | 同左（参数接口复用） |

## 注意

- 真机串口仍可能被 `robot.service` 占用，`run_hw.sh` 会走 `claim_arm_serial.sh`（可能停掉机载 `robot.launch` **及其附带的 VRPN**）。因此本项目 **默认自启** `~/zihan_ws/vicon_perception` 的 VRPN，不要假设系统 VRPN 仍在。
- 先模式 **A**，确认 \(\Delta\) 与补偿方向符合标定后再试 B/E/C。
- 若标定中飞机与臂座还有固定外参，应在标定阶段并入；本桥接只使用**相对** \(\Delta\)，不依赖 Vicon 绝对原点。
- 模式 **E**：在 Mode B（IK+CTC）上用 CLIK 积分生成连续 \(q^*\)、零空间贴上一帧、任务死区、IK 跳变门控与软速率限制；**关闭**固定 `q_des` 低通，力矩侧轻微 LPF。用于在防抖的同时减小相位滞后。
- 模式 **C（偏稳消抖）**：MIT 钉住测得 `q`；`hw_zero_dq` 关闭速度通道；OSC 力矩经 `hw_torque_lpf_alpha` 低通后再限幅；不叠加 IK 位置追赶。
- bridge 若长时间收不到 pose，会周期性打 WARN（`No messages on /vrpn/...`）；此时控制端不会有自稳反应。