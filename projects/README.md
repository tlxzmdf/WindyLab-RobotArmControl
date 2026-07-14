# projects/

各子目录对应一个独立应用/实验项目。与 ROS 包解耦：包在 `../windylab_ws/src/`，项目资产在此。

| 目录 | 名称 |
|------|------|
| [`ee-stabilization/`](ee-stabilization/) | 机头稳定（末端世界系位姿保持） |
| [`circle-draw/`](circle-draw/) | 末端画圆轨迹优化（微分 IK + 速度前馈） |
| [`master-slave-stabilization/`](master-slave-stabilization/) | 真主臂遥操作仿真从臂 + 机载端扰动 + 末端自稳（CLIK/IK） |
| [`master-slave-wbc/`](master-slave-wbc/) | 同上场景，控制重构为 **QP-WBC + 积分动作** |

新建项目时复制 `ee-stabilization/` 的目录骨架即可。
