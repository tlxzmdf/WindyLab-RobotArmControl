#!/usr/bin/env python3
"""仿真：先 VRPN / bridge latch，再启动 ee_stabilization(base_source:=external)。"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    project_dir = Path(__file__).resolve().parents[1]
    bridge_py = str(project_dir / 'scripts' / 'vicon_relative_bridge.py')

    package_share = Path(get_package_share_directory('arm_ee_stabilization_description'))
    control_share = Path(get_package_share_directory('arm_ee_stabilization_control'))
    display_urdf = package_share / 'urdf' / 'arm_on_drone.urdf'
    ik_urdf = package_share / 'urdf' / 'single_arm.urdf'
    rviz_config = package_share / 'rviz' / 'stabilization.rviz'
    control_params = control_share / 'config' / 'stabilization.yaml'
    robot_description = display_urdf.read_text(encoding='utf-8')

    pose_topic = LaunchConfiguration('pose_topic').perform(context)
    latch_delay = float(LaunchConfiguration('latch_delay').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in (
        'true', '1', 'yes',
    )
    start_vrpn = LaunchConfiguration('start_vrpn').perform(context).lower() in (
        'true', '1', 'yes',
    )
    vicon_ws = LaunchConfiguration('vicon_ws').perform(context)

    actions = []

    # 1) VRPN（可选）
    if start_vrpn:
        vrpn_setup = os.path.join(vicon_ws, 'install', 'setup.bash')
        if os.path.isfile(vrpn_setup):
            actions.append(
                ExecuteProcess(
                    cmd=[
                        'bash', '-lc',
                        f'source /opt/ros/humble/setup.bash && '
                        f'source "{vrpn_setup}" && '
                        f'ros2 launch vrpn_listener vrpn_client.launch',
                    ],
                    output='screen',
                )
            )
        else:
            print(f'[WARN] vicon ws setup not found: {vrpn_setup}')

    # 2) RSP + bridge（VRPN 发现 tracker 通常再等几秒）
    bridge_start = 3.0 if start_vrpn else 0.2
    tracker_slack = 4.0 if start_vrpn else 0.0
    actions.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 500.0,
                'use_sim_time': False,
            }],
        )
    )
    actions.append(
        TimerAction(
            period=bridge_start,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'python3', bridge_py,
                        '--pose-topic', pose_topic,
                        '--latch-delay', str(latch_delay),
                        '--no-tf',
                    ],
                    output='screen',
                )
            ],
        )
    )

    # 3) 稳定节点：等 bridge 完成 latch 后再锁末端
    stab_delay = bridge_start + tracker_slack + latch_delay + 1.0
    stab_node = Node(
        package='arm_ee_stabilization_control',
        executable='ee_stabilization',
        name='ee_stabilization',
        output='screen',
        parameters=[
            str(control_params),
            {
                'urdf_path': str(ik_urdf),
                'base_source': 'external',
                'hardware_mode': False,
            },
        ],
    )
    actions.append(TimerAction(period=stab_delay, actions=[stab_node]))

    if use_rviz:
        actions.append(
            TimerAction(
                period=stab_delay + 1.5,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        arguments=['-d', str(rviz_config)],
                        output='screen',
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    default_vicon = os.path.expanduser('~/zihan_ws/vicon_perception/src')
    return LaunchDescription([
        DeclareLaunchArgument('pose_topic', default_value='/vrpn/pregme/pose'),
        DeclareLaunchArgument('latch_delay', default_value='2.0'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('start_vrpn', default_value='False'),
        DeclareLaunchArgument('vicon_ws', default_value=default_vicon),
        OpaqueFunction(function=_setup),
    ])
