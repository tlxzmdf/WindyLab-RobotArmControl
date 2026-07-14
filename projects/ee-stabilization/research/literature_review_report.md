# 浮动基座机械臂末端世界系位姿稳定 — 文献与开源调研报告

**调研对象：** `ee-stabilization` 项目（A-L1-GAMMA 六轴 / Pinocchio / ROS2）  
**调研日期：** 2026-07-08  
**报告作者：** 自动化文献调研（基于公开论文与开源仓库）  
**资源目录：** [`bibliography.md`](bibliography.md) | [`projects/open_source_index.md`](projects/open_source_index.md) | [`papers/`](papers/)

---

## 执行摘要

本报告围绕 **「时变浮动基座上，机械臂仅通过关节控制，使末端在世界惯性系中保持固定位姿」** 这一核心问题，对 2020–2026 年相关论文与开源项目进行了系统检索与对比分析。

**主要结论：**

1. **问题定位清晰且研究活跃**：该问题在 **空中机械臂（Aerial Manipulation）**、**自由漂浮空间机械臂**、**移动/人形 loco-manipulation** 三条线路上均有大量工作；与本项目 **「T_B 外生、仅臂补偿、世界系锁定 T_E*」** 的 formulation 高度一致（见 `docs/problem_formulation.md`）。

2. **方法可归纳为六大类**：运动学/CLIK、广义雅可比（GJM）、关节/任务空间动力学控制（CTC/OSC）、全身 QP 多任务、扰动观测/鲁棒控制、学习/分层策略。本项目已覆盖前四类中的 **A/B/C 三模式**，在同类开源实现中属于 **结构完整、可对比性强** 的工程方案。

3. **与文献的异同**：
   - **同**：世界系目标 → 基座系时变期望 `T_D = T_B^{-1} T_E*`；CLIK + DLS；OSC 作为任务空间力矩方案；Pinocchio 动力学。
   - **异**：多数空中机械臂论文 **联合控制 UAV+臂**（耦合 WBC/QP）；本项目 **刻意不解控 UAV**，更贴近「机载臂被动承受 mount 扰动」的台架/任务假设；文献中 **GJM/零反力矩** 针对冗余臂，本项目 6DOF 非冗余，自由度利用空间较小。

4. **对本项目的建议优先级**：
   - **短期（真机）**：继续以 **模式 C（OSC）** 为主，补充 **CBF/软限位**（参考 oscbf）、**T_B 估计延迟** 敏感性测试。
   - **中期（算法）**：引入 **Wang 2024 式关节速度前馈规划器** 与现有 CLIK 对比；评估 **加速度级 CLIK**（Ruggiero 2024）对高频扰动的收益。
   - **长期（扩展）**：若需 UAV 本体参与补偿，可迁移 **RAL 2019 / arXiv 2601 QP** 全身任务栈；若 `base_source=tf` 且基座未知，参考 **ManipulationOnTheMove** 反应式架构。

---

## 1. 问题定义与调研范围

### 1.1 本项目问题（对照基准）

根据 `docs/problem_formulation.md` 与 `docs/main.tex`：

| 要素 | 本项目设定 |
|------|------------|
| 基座 | `{B}` 随 UAV/mount 时变，`T_B(t)` **外生、不可测控** |
| 任务 | 启动时锁定 `T_E* ∈ SE(3)`，之后 **世界系位姿恒定** |
| 等价跟踪 | 基座系期望 `T_D(t) = T_B(t)^{-1} T_E*`（时变） |
| 控制输出 | 关节力矩/位置（MIT 阻抗），**不控制旋翼/ mount** |
| 评价 | `\|p_E^W - p_E*\|`, `\|Log(R_E^W^T R_E*)\|`, `\|e_x\|` |
| 实现模式 | A: 纯 IK；B: IK+CTC；C: IK+OSC |

### 1.2 调研检索策略

- **关键词**：floating-base manipulator, aerial manipulator, end-effector stabilization, world-frame tracking, generalized Jacobian, CLIK, operational space control, whole-body QP, base motion compensation。
- **来源**：IEEE Xplore、arXiv、MDPI、Frontiers、Nature Sci Reports、GitHub。
- **时间侧重**：2020–2026，兼顾 GJM/OSC 经典文献。
- **下载**：PDF 已存于 `research/papers/`（见 bibliography）；含 **Wang et al. 2024 TASE (P1)** 及多篇开放获取文献。

### 1.3 不在本次范围

- 载体（UAV）姿态/位置控制律设计（除非作为对比的 **耦合方案**）。
- 时变轨迹跟踪 `T_E*(t)`（抓取移动目标等）。
- 接触力控、视觉伺服完整系统（仅讨论其与定点稳定的关系）。

---

## 2. 方法分类与代表工作

```text
                    ┌─────────────────────────────────────┐
                    │   世界系目标 T_E* (常值)              │
                    └─────────────────┬───────────────────┘
                                      │ T_D = T_B^{-1} T_E*
                    ┌─────────────────▼───────────────────┐
  规划/运动学层      │ CLIK / DLS-IK / GJM / 速度规划器     │
                    └─────────────────┬───────────────────┘
                                      │ q_D, q̇_D
          ┌───────────────────────────┼───────────────────────────┐
          ▼                           ▼                           ▼
     模式 A: 直接设 q            模式 B: CTC τ(q)            模式 C: OSC τ(e_x)
          │                           │                           │
          └───────────────────────────┴───────────────────────────┘
                                      │ τ / MIT 指令
                    ┌─────────────────▼───────────────────┐
  执行层             │  电机 / Pinocchio ABA 仿真           │
                    └─────────────────────────────────────┘

  扩展路线（文献常见，本项目未实现）：
  · 全身 QP：基座 wrench + 关节 τ 联合优化
  · NDO/ADRC：扰动观测补偿
  · 零空间：冗余 DOF 最小化反力矩/CoG 偏移
  · RL/分层：HiWET 类世界系 subgoal
```

### 2.1 运动学 / CLIK / 阻尼 IK

**思想**：每周期由 `T_D` 求 `q_D`；CLIK 用 `dq = J^T (JJ^T + λ²I)^{-1}(K⊙e + v_task)` 提供速度前馈与误差反馈。

| 代表文献 | 核心贡献 | 优缺点 |
|----------|----------|--------|
| 本项目模式 A | 评估规划链几何上界；标称 pos RMS **0.28 mm** | ✅ 实现简单、可解释；❌ 无动力学，非真机方案 |
| Ruggiero et al. 2024 (P2) | 速度/加速度级 CLIK，GJM 适配 UAM | ✅ 系统处理风扰/参数误差；❌ 需准确 `T_B` 与模型 |
| Wang et al. 2024 (P1) | **关节速度规划器**专门抵消基座浮动 | ✅ 与「反向补偿 mount」直觉一致，有实验；❌ 需 NN+NDO 补动力学 |

**与本项目关系**：模式 A 的极限实验（196 runs）表明 **IK+T_D 变换在 scale≤1.4 下 pos RMS 仍 <1.4 mm**，与文献一致：**运动学层不是主要瓶颈**，性能差异来自控制层与执行层。

### 2.2 广义雅可比矩阵（GJM）

**思想**：自由漂浮系统中，末端速度不仅依赖 `\dot{q}`，还与基座速度耦合；GJM 在 **动量守恒**（空间）或 **外力已知**（空中）下建立 `\dot{x}_{EE} = J^* \dot{q}`。

| 代表文献 | 场景 | 要点 |
|----------|------|------|
| Umetani & Yoshida 1989; Papadopoulos 1993 (C1–C2) | 空间 | 奠基；**动态奇异**（路径依赖） |
| Stolt et al. 2018 (C3) | 空间 | 工程建模教程，符号 GJM |
| Orsag et al. 2022 (P4) | 空中 | **扩展 GJM**：计入重力与 UAV 控制力，非动量守恒 |
| Ruggiero 2025 (P3) | 空中 | 加速度级 + 自由关节 |

**与本项目差异**：

- 本项目 **6DOF 非冗余**（任务 6D），GJM 的冗余零空间（反力矩最小化）**无法直接使用**。
- 本项目 **基座由外源驱动**（mount 轨迹），而非动量耦合推导出的被动运动；更接近 **「已知 T_B 的 CLIK」** 而非经典 free-floating SMS。
- **可借鉴**：P4 将 **UAV 控制外力** 纳入 IK——若未来 mount 动力学可测，可提升高频段预测精度。

### 2.3 动力学控制：CTC vs OSC

| 维度 | CTC（模式 B） | OSC（模式 C） |
|------|---------------|---------------|
| 控制空间 | 关节空间 `q → q_D` | 任务空间 `e_x → 0` |
| 与评价指标 | **不一致**（关节 vs 末端） | **一致** |
| 本项目标称 pos RMS | 2.81 mm | **0.36 mm** |
| 极限实验 | 最早失效（229 mm @ S1） | 居中，标称 scale=1 即 FAIL |
| 文献共识 | 关节跟踪常用 | **空中/浮动基座末端任务首选**（P5, Sentis 2005） |

**文献支撑**：

- **Sentis & Khatib 2005 (C4)**：扩展 GJM + 操作空间任务优先级——浮动基座 OSC 的理论源头之一。
- **Forte et al. RAL 2019 (P5)**：多任务 QP 在任务空间定义末端/力/姿态，验证 **任务空间 formulation** 对空中机械臂交互的重要性。
- **Stanford oscbf (开源)**：现代 kHz OSC + CBF，**显式支持移动基座链**（PPR 关节接在链首）。

**结论**：本项目实验与文献一致——**OSC 更匹配世界系位姿稳定**；CTC 适合作为「关节执行器带宽/力矩饱和」诊断工具，不宜作部署方案。

### 2.4 全身二次规划（Whole-Body QP）与分层任务

**思想**：将基座（6DOF 或 4DOF 欠驱动）与臂关节置于 **统一 QP**，优先级栈：末端位姿 > 基座稳定 > 关节限位 > 反力矩最小化。

| 代表 | 控制对象 | 特点 |
|------|----------|------|
| Forte RAL 2019 (P5) | UAV + 臂 | 力/位混合；FT 传感器 |
| arXiv 2601.08523 (P6) | 欠驱动四旋翼 + 臂 | 约束显式、积分抗扰 |
| Lippiello 2015; Centropiaggio SAM (D3–D4) | 缆悬 AM | 末端最高优先级，基座阻尼次级 |
| am-planner (开源) | 规划层 whole-body | 轨迹优化，非定点稳定 |

**与本项目对比**：

| | 本项目 | QP/WBC 文献 |
|--|--------|-------------|
| UAV/mount 控制 | ❌ 外生 | ✅ 联合优化 |
| 实现复杂度 | 低 | 高（QP 求解器、约束建模） |
| 扰动能力上限 | 受臂 workspace/力矩限 | 基座+臂 **可分担** 大扰动 |
| 适用场景 | 机载臂 **不接管飞控** | 一体化 aerial manipulator |

**建议**：若扰动包络超出臂单独补偿能力（见 limit_test FAIL 边界），可评估 **「软耦合」**：mount 轨迹预测 + 臂 OSC，而非全 QP。

### 2.5 扰动观测与鲁棒控制

| 方法 | 代表 | 适用 |
|------|------|------|
| NDO + NN | Wang 2024 (P1) | 复合扰动、模型误差 |
| ADRC + 反步 | Nature 2021 (E1) | 全驱动 AMS，Cartesian 控制 |
| H∞ + 加速度反馈 | MDPI 2019 (E2) | 风扰，**UAV 层**为主 |
| 级联动态补偿 | Robotica 2023 (E3) | 速度级参考 + 动力学内环 |

**共同点**：文献多在 **动力学环** 补扰动；本项目 OSC 已含 `\Lambda, μ` 前馈，但 **未显式 NDO/ESO**。

**建议**：在模式 C 上叠加 **轻量 ESO**（估计 `\dot{e}_x` 或 `\tau_{dist}`）成本低，与 Wang 2024 的「规划+观测」架构互补。

### 2.6 解耦 vs 耦合控制

```text
解耦（Decoupled）                    耦合（Coupled / Whole-body）
─────────────────────                ─────────────────────────────
UAV: 姿态/位置控制器                  统一动力学 + QP
Arm: IK/OSC 补偿 T_B                 基座 wrench + τ 同时求解
─────────────────────                ─────────────────────────────
本项目 ✓                             RAL 2019, arXiv 2601 ✓
Chen 2023 CoG 零空间 ✓               OAM arXiv 2508 ✓
```

**文献倾向**：

- **解耦**：实现快、安全（不改飞控）；性能受 **臂 workspace、力矩、T_B 带宽** 限制——**正是本项目选型**。
- **耦合**：扰动裕度大，但 **飞控-臂耦合调试复杂**，且与本项目「mount 外生」假设冲突。

### 2.7 学习与世界系推理（相邻）

- **HiWET (F1, arXiv 2602)**：人形 **世界系 EE 跟踪**，高层 subgoal + 低层稳定；强调 **body-centric 方法无法纠正 world-frame 漂移**——与本项目 world-frame locking 原则 **完全一致**，但方法为 RL，非模型控制。
- **am-planner IL 引导优化**：规划层用学习克服局部最优，控制层仍传统。

**评价**：学习方案数据需求大、可解释性弱；**不建议替代** 当前 OSC 主方案，可作为 **长时域 loco-manipulation** 扩展参考。

---

## 3. 开源项目对比

详见 [`projects/open_source_index.md`](projects/open_source_index.md)。

| 项目 | 场景相似度 | 可复用组件 | 差距 |
|------|------------|------------|------|
| **ManipulationOnTheMove** | ★★★★★ | 无先验基座扰动下的反应式 gripper 稳定 | 移动底盘非 UAV；ROS1 |
| **oscbf** | ★★★★ | OSC + CBF 安全滤波、移动基座链 | 无「世界系锁定」专用逻辑 |
| **wbc_ros** | ★★★★ | ROS2 WBC 任务定义 | 需自行添加 SE(3) 锁定任务 |
| **upright/OCS2** | ★★★ | 约束 MPC 框架 | 任务为托盘平衡，非定点 |
| **am-planner** | ★★★★ | 全身轨迹、waypoint 约束 | 偏规划，ROS Noetic |
| **本项目 ee-stabilization** | — | **A/B/C 对比 + 极限扫参 + 真机 MIT 链路** | 无 QP/NDO/冗余零空间 |

**独特优势（本项目）**：

1. **Problem formulation 文档化**清晰，与文献符号对齐。
2. **三模式 + limit_test 196 点** 提供可复现的性能边界数据。
3. **仿真-真机 parity**（`base_source` simulated/static/tf）。

---

## 4. 系统对比矩阵

### 4.1 方法维度

| 方法 | 末端精度潜力 | 实现难度 | 需 UAV 控制 | 需冗余 DOF | 对 T_B 精度敏感度 | 真机可部署性 |
|------|-------------|----------|-------------|------------|-------------------|--------------|
| 纯 IK / A | 中（几何上界） | ★ | 否 | 否 | 高 | 低（无力矩） |
| CLIK + 速度规划 (P1) | 高 | ★★ | 否 | 否 | 高 | 中 |
| CTC / B | 中-低 | ★★ | 否 | 否 | 中 | 中 |
| **OSC / C** | **高** | ★★★ | 否 | 否 | 高 | **高** |
| GJM + 加速度 IK | 高 | ★★★★ | 否 | 优 | 高 | 中 |
| 全身 QP | 很高 | ★★★★★ | **是** | 优 | 中 | 低 |
| NDO/ADRC 叠加 | 高+ | ★★★ | 可选 | 否 | 中 | 中 |
| RL (HiWET) | 很高（数据够） | ★★★★★ | 是 | — | 低-中 | 低 |

### 4.2 与本项目实验的对照解读

来自 `reports/limit_test/summary/experiment_report.md`：

| 发现 | 文献解释 |
|------|----------|
| A 全程未 FAIL | CLIK+T_D 在扰动包络内 **运动学可行**；与 P2 仿真结论一致 |
| B 平移+高频最先失效 | 关节空间目标与任务空间指标失配 + LPF 滞后 → 文献较少推荐 CTC 作 EE 任务 |
| C 标称 scale=1 FAIL | OSC 增益/力矩限/`\Lambda` 奇异性需整定；RAL 2019 强调 **硬约束 QP** 处理饱和 |
| 纯旋转 S2 下 C 优于 B | 姿态误差 `\|e_R\|` 更依赖任务空间 `\Lambda` 耦合，OSC 有优势 |

---

## 5. 研究空白与趋势（2024–2026）

1. **仅臂补偿 + 世界系定点稳定** 的 **标准化 benchmark** 仍少；多数论文做 **轨迹跟踪** 或 **抓取**，定点稳定多作为子模块。
2. **T_B 估计延迟/噪声** 对 CLIK/OSC 的 **定量灵敏度分析** 在文献中不足——本项目 `base_source=tf` 真机场景值得发 paper。
3. **非冗余 6DOF 臂** 在大幅度 mount 扰动下的 **workspace 边界**（非控制律问题）与 limit_test 数据可填补空白。
4. **趋势**：全向 AM (OAM)、全身 NMPC/QP、learning 世界系 subgoal 增多；**经典 OSC+CLIK 仍是工业可部署主流**。

---

## 6. 对本项目的具体建议

### 6.1 短期（1–2 周，真机强化）

| 优先级 | 动作 | 参考 |
|--------|------|------|
| P0 | 保持 **模式 C**，完善 OSC 增益与 `\tau` 限幅 | 本项目 limit_test |
| P1 | 增加 **T_B 延迟/噪声注入测试**（simulated mount） | MotM 反应式场景 |
| P2 | 引入 **软限位 CBF** 或 Pinocchio 关节 barrier | oscbf |
| P3 | 文档化 **FAIL 边界** 为「最大可补偿扰动包络」 | 对标 P1 实验协议 |

### 6.2 中期（1–2 月，算法对比）

| 优先级 | 动作 | 参考 |
|--------|------|------|
| P1 | 实现 **Wang 2024 关节速度规划器** 作为 CLIK 替代/叠加 | P1 |
| P2 | 试 **加速度级 CLIK**（Ruggiero 2024） | P2, P3 |
| P3 | OSC 上叠 **ESO/NDO** 估计扰动力矩 | P1, E1 |
| P4 | 评估 **Liegeois 零空间** 偏向舒适位形（已有 API 注释） | PinocchioDynamicsModel.hpp |

### 6.3 长期（架构扩展）

- **若 mount 可控**：引入 **wbc_ros** 或 arXiv 2601 QP，定义最低优先级任务「末端 world pose」。
- **若仅 TF 无先验**：参考 **ManipulationOnTheMove** 纯反馈架构。
- **若改冗余臂**：GJM + CoG 零空间（P4, D1）降低对 mount 反力矩。

---

## 7. 结论

浮动基座末端世界系位姿稳定是一个 **建模清晰、工程可落地、研究持续活跃** 的方向。本项目在 **问题形式化、世界系锁定、三模式可对比架构、极限实验** 方面已达到 **研究型工程** 水准。

与最新文献相比：

- **理论路线正确**：`T_D = T_B^{-1} T_E*` + CLIK + OSC 是主流范式（P2, P5, C4）。
- **差异化明确**：**仅臂补偿、不控 UAV**，区别于大多数 aerial manipulation 全身控制论文。
- **主要提升空间**：在 **模式 C** 上加强扰动观测、安全约束、T_B 鲁棒性；可选引入 **速度规划器（P1）** 与 **加速度级 IK（P3）**；长期视需求考虑 **QP 耦合** 或 **反应式无模型** 分支。

---

## 附录 A：已下载 PDF 清单

| 文件名 | 大小 | 说明 |
|--------|------|------|
| `Precise_End-Effector_Control_..._Experiments.pdf` | 4.6 MB | **Wang 2024 TASE (P1)**，关节速度规划 + NDO |
| `arXiv_2508_19608_OAM_Robust_Control.pdf` | 14 MB | 全向 AM 鲁棒控制 |
| `arXiv_2602_06341_HiWET_Humanoid.pdf` | 5.6 MB | 世界系 EE 分层 RL |
| `arXiv_2508_15732_Dynamic_Coupling_Space.pdf` | 2.3 MB | 空间机械臂动态耦合 |
| `arXiv_2601_08523_QP_Underactuated_AM.pdf` | 3.5 MB | 欠驱动 AM QP |
| `Frontiers2018_FreeFloating_SMS_Tutorial.pdf` | 4.1 MB | GJM 教程 |
| `Sentis2005_FreeFloating_Humanoid_ICRA.pdf` | 269 KB | 浮动基座 OSC |
| `Dynamic_Singularities_FreeFloating_1993.pdf` | 123 KB | 动态奇异 |
| `Nature2021_ADRC_Backstepping.pdf` | 6.3 MB | ADRC 补偿 |
| `MDPI2019_Aerial_Grasping_Wind.pdf` | 3.9 MB | 风扰分层控制 |
| `Centropiaggio_SAM_WholeBody.pdf` | 6.0 MB | 缆悬 AM 全身控制 |

## 附录 B：建议优先精读（按相关性排序）

1. **Wang et al. 2024 TASE (P1)** — 关节速度规划 + NDO，与项目场景几乎一一对应  
2. **Ruggiero et al. 2024 Appl. Sci. (P2)** — CLIK 速度/加速度级完整流程  
3. **Sentis & Khatib 2005 (C4)** — 浮动基座 OSC 理论  
4. **Forte et al. RAL 2019 (P5)** — 多任务 QP 空中机械臂  
5. **Stolt et al. 2018 (C3)** — GJM 建模（理解空间→空中迁移）  
6. **ManipulationOnTheMove 项目页** — 反应式 world-space gripper 稳定  

---

*本报告基于公开文献与开源资料整理。P1 (Wang 2024 TASE) 全文已存于 `papers/`。后续可在 `research/notes/` 中追加单篇精读笔记。*
