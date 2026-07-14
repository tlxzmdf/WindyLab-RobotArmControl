# 文献与资源目录

> 调研日期：2026-07-08  
> PDF 存放路径：`papers/`  
> 标注：✅ 已下载 PDF | 🔗 仅在线摘要/页面 | 🔒 付费墙

---

## A. 与本项目问题最直接相关（浮动基座 + 世界系末端保持/跟踪）

| ID | 文献 | 年份 | 方法要点 | 本地文件 | DOI/arXiv |
|----|------|------|----------|----------|-----------|
| P1 | **Wang et al.** Precise End-Effector Control for an Aerial Manipulator Under Composite Disturbances | 2024 | 关节速度规划器抵消基座浮动 + NN/NDO 动力学补偿 | ✅ `Precise_End-Effector_Control_for_an_Aerial_Manipulator_Under_Composite_Disturbances_Theory_and_Experiments.pdf` | [10.1109/TASE.2024.3406754](https://doi.org/10.1109/tase.2024.3406754) |
| P2 | **Ruggiero et al.** Trajectory Tracking Control of an Aerial Manipulator in the Presence of Disturbances and Model Uncertainties | 2024 | CLIK 速度/加速度级 IK，Generalized Jacobian 适配 UAM | 🔗 | [10.3390/app14062512](https://doi.org/10.3390/app14062512) |
| P3 | **Ruggiero et al.** Inverse Kinematics of a Serial Manipulator with a Free Joint for Aerial Manipulation | 2025 | 加速度级 GJM + 外力的 IK | 🔗 | [10.3390/applsci15058390](https://doi.org/10.3390/applsci15058390) |
| P4 | **Orsag et al.** Zero Reaction Torque Trajectory Tracking through Extended Generalized Jacobian | 2022 | 扩展 GJM，最小化对 UAV 反力矩 | 🔗 | [10.3390/app122312254](https://doi.org/10.3390/app122312254) |
| P5 | **Forte et al.** Direct Force Feedback Control and Online Multi-Task Optimization for Aerial Manipulators | 2019 | 全身 QP 多任务，人形 WBC 迁移至空中机械臂 | 🔗 | [10.1109/LRA.2019.2958473](https://doi.org/10.1109/lra.2019.2958473) |
| P6 | **arXiv 2601.08523** QP-Based Control of an Underactuated Aerial Manipulator under Constraints | 2026 | 欠驱动 AM 全身 QP，基座+末端联合跟踪 | ✅ `arXiv_2601_08523_QP_Underactuated_AM.pdf` | [2601.08523](https://arxiv.org/abs/2601.08523) |
| P7 | **arXiv 2508.19608** Autonomous Aerial Manipulation at Arbitrary Pose in SE(3) | 2025 | 全向 AM 几何鲁棒控制 + 全身规划 | ✅ `arXiv_2508_19608_OAM_Robust_Control.pdf` | [2508.19608](https://arxiv.org/abs/2508.19608) |

## B. 经典理论基础（自由漂浮 / 广义雅可比 / OSC）

| ID | 文献 | 年份 | 方法要点 | 本地文件 |
|----|------|------|----------|----------|
| C1 | **Umetani & Yoshida** Resolved Motion Rate Control of Space Manipulators (Generalized Jacobian) | 1989 | GJM 奠基 | — |
| C2 | **Papadopoulos & Dubowsky** Dynamic Singularities in Free-Floating Space Manipulators | 1993 | 动态奇异 | ✅ `Dynamic_Singularities_FreeFloating_1993.pdf` |
| C3 | **Stolt et al.** Equations of Motion of Free-Floating Spacecraft-Manipulator Systems: An Engineer's Tutorial | 2018 | GJM 建模教程 | ✅ `Frontiers2018_FreeFloating_SMS_Tutorial.pdf` |
| C4 | **Sentis & Khatib** Control of Free-Floating Humanoid Robots | 2005 | 扩展 GJM + 任务优先级 OSC | ✅ `Sentis2005_FreeFloating_Humanoid_ICRA.pdf` |
| C5 | **arXiv 2508.15732** Understanding and Utilizing Dynamic Coupling in Free-Floating Space Manipulators | 2025 | 动态耦合因子、在轨服务 | ✅ `arXiv_2508_15732_Dynamic_Coupling_Space.pdf` |

## C. 解耦控制 / 仅臂补偿 / 冗余优化

| ID | 文献 | 年份 | 方法要点 | 本地文件 |
|----|------|------|----------|----------|
| D1 | **Chen et al.** Null-Space Minimization of CoG Displacement of a Redundant Aerial Manipulator | 2023 | 解耦 UAV/臂，零空间最小化 CoG 位移 | 🔗 |
| D2 | **Chen et al.** Dynamic Grasping Based on Coupling Disturbance Compensation | 2023 | IK 末端位置补偿 + 耦合扰动补偿 | 🔗 |
| D3 | **Lippiello et al.** Hybrid Visual Servoing With Hierarchical Task Composition | 2015 | 分层任务：末端 + 基座 + 冗余 | 🔗 |
| D4 | **Centropiaggio SAM** Whole-body dynamically-decoupling control | — | 末端最高优先级，基座振荡阻尼次级 | ✅ `Centropiaggio_SAM_WholeBody.pdf` |

## D. 扰动观测 / 鲁棒控制

| ID | 文献 | 年份 | 方法要点 | 本地文件 |
|----|------|------|----------|----------|
| E1 | **Nature Sci Reports** ADRC-Backstepping Compensation for AMS | 2021 | ADRC + 反步 + 动力学补偿 | ✅ `Nature2021_ADRC_Backstepping.pdf` |
| E2 | **MDPI Appl. Sci.** Aerial Grasping in Strong Wind (H∞ + 分层解耦 IK) | 2019 | UAV H∞ + 臂 IK 解耦 | ✅ `MDPI2019_Aerial_Grasping_Wind.pdf` |
| E3 | **Robotica 2023** Multitask control with dynamic compensation (numerical) | 2023 | 级联：最小范数运动学 + 动态补偿 | 🔗 |

## E. 相邻领域（世界系末端跟踪 / 移动操作）

| ID | 文献 | 年份 | 方法要点 | 本地文件 |
|----|------|------|----------|----------|
| F1 | **HiWET** Hierarchical World-Frame End-Effector Tracking for Humanoid Loco-Manipulation | 2026 | 世界系 EE 跟踪 + 分层 RL | ✅ `arXiv_2602_06341_HiWET_Humanoid.pdf` |
| F2 | **ManipulationOnTheMove** Reactive architecture for mobile manipulation on-the-move | 2020s | 无先验基座扰动，反应式稳定 gripper | 见 `projects/` |
| F3 | **Drones 2025** Review of Real-Time Cooperative Aerial Manipulation Systems | 2025 | 综述：原型、建模、控制对比 | 🔗 |

## F. 本项目内部文档

| 文档 | 路径 |
|------|------|
| Problem Formulation | `../docs/problem_formulation.md` |
| Design Specification | `../docs/main.tex` |
| 极限实验报告 | `../reports/limit_test/summary/experiment_report.md` |
| 三模式测试 | `../reports/mode_test_report.md` |

---

## 下载说明

- **已成功下载**（`papers/` 目录）：**IEEE TASE 2024 (P1)**、arXiv 论文 4 篇、Frontiers 教程、Sentis ICRA、动态奇异经典文、Nature ADRC、Centropiaggio SAM、MDPI 2019 风扰抓取。
- **MDPI 2022–2025 若干篇**：站点对自动化下载返回 HTML 拦截页，已在调研报告中引用摘要与 DOI；建议通过机构 VPN 或浏览器手动下载。
