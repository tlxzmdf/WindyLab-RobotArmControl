# projects/

各子目录对应一个独立应用/实验项目。与 ROS 包解耦：包在 `../windylab_ws/src/`，项目资产（文档、启动脚本、配置、报告）在此。

| 目录 | 名称 | 说明 |
|------|------|------|
| [`circle-draw/`](circle-draw/) | 末端画圆 | 微分 IK / 预计算轨迹，平滑画圆 |
| [`ee-stabilization/`](ee-stabilization/) | 机头稳定 | UAV mount 扰动下末端世界系位姿保持 |
| [`master-slave-stabilization/`](master-slave-stabilization/) | 主从自稳 (CLIK) | 真主臂 → 仿真从臂；关节镜像或 CLIK |
| [`master-slave-wbc/`](master-slave-wbc/) | 主从自稳 (WBC) | 同上场景，控制为 QP-WBC + 积分动作 |
| [`teleop-compare/`](teleop-compare/) | CLIK vs WBC 对比 | 公平参数下录制、曲线与急动分析 |
| [`reports/`](reports/) | 跨项目报告 | 可选的汇总对比输出 |

工作区总览与编译、电脑版脚本见 [`../README.md`](../README.md)。新建项目时复制 `ee-stabilization/` 的目录骨架，并更新上表与根 README。
