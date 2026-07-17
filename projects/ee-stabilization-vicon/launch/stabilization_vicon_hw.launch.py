#!/usr/bin/env python3
"""ee-stabilization-vicon 真机 launch：先回 home，再 Vicon 相对扰动 + 稳定。

启动顺序：
  0) （可选，默认开）vrpn_listener from vicon_ws — claim 串口后常需自启
  1) student_arm + robot_state_publisher
  2) move_to_home（余弦插值到 q_home）
  3) vicon_relative_bridge（latch t0，发 world→base_link）
  4) ee_stabilization
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    project_dir = Path(__file__).resolve().parents[1]
    bridge_py = str(project_dir / 'scripts' / 'vicon_relative_bridge.py')
    home_py = str(project_dir / 'scripts' / 'move_to_home.py')

    pose_topic = LaunchConfiguration('pose_topic').perform(context)
    latch_delay = float(LaunchConfiguration('latch_delay').perform(context))
    offset_z = LaunchConfiguration('mount_base_offset_z').perform(context)
    start_vrpn = LaunchConfiguration('start_vrpn').perform(context).lower() in (
        'true', '1', 'yes',
    )
    vicon_ws = LaunchConfiguration('vicon_ws').perform(context)
    home_before = LaunchConfiguration('home_before_stabilize').perform(context).lower() in (
        'true', '1', 'yes',
    )
    home_duration = float(LaunchConfiguration('home_duration').perform(context))
    home_settle = float(LaunchConfiguration('home_settle').perform(context))
    arm_type = LaunchConfiguration('arm_type').perform(context)
    port_name = LaunchConfiguration('port_name').perform(context)
    max_velocity = float(LaunchConfiguration('max_velocity').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in (
        'true', '1', 'yes',
    )
    selected = LaunchConfiguration('stabilization_mode').perform(context).upper()
    params_overlay = LaunchConfiguration('params_overlay').perform(context).strip()

    control_share = Path(get_package_share_directory('arm_ee_stabilization_control'))
    desc_share = Path(get_package_share_directory('arm_ee_stabilization_description'))
    manipulator_share = Path(get_package_share_directory('manipulator'))

    mode_files = {
        'A': control_share / 'config' / 'stabilization_hw_mode_a.yaml',
        'B': control_share / 'config' / 'stabilization_hw_mode_b.yaml',
        'C': control_share / 'config' / 'stabilization_hw_mode_c.yaml',
        'D': control_share / 'config' / 'stabilization_hw_mode_d.yaml',
        'E': control_share / 'config' / 'stabilization_hw_mode_e.yaml',
    }
    mode_yaml_path = str(mode_files.get(selected, mode_files['A']))
    base_params = control_share / 'config' / 'stabilization.yaml'
    ik_urdf = desc_share / 'urdf' / 'single_arm.urdf'
    student_cfg = manipulator_share / 'stabilization_hw_student_arm.yaml'
    motor_config_path = str(manipulator_share / 'motor_config.yaml')
    arm_config_path = str(manipulator_share / 'arm_config.yaml')
    rviz_config = desc_share / 'rviz' / 'stabilization.rviz'

    urdf_path = os.path.join(str(manipulator_share), 'arm.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    actions = []

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
                    name='vrpn_listener',
                )
            )
        else:
            print(f'[WARN] vicon ws setup not found: {vrpn_setup}; skip start_vrpn')

    student_arm = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            str(student_cfg),
            {
                'arm_type': arm_type,
                'arm_version': 'gamma',
                'port_name': port_name,
                'max_velocity': max_velocity,
                'motor_config_path': motor_config_path,
                'arm_config_path': arm_config_path,
            },
        ],
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    actions.extend([student_arm, rsp])

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            output='screen',
        ))

    bridge = ExecuteProcess(
        cmd=[
            'python3', bridge_py,
            '--pose-topic', pose_topic,
            '--mount-base-offset-z', offset_z,
            '--latch-delay', str(latch_delay),
        ],
        output='screen',
        name='vicon_relative_bridge',
    )
    ee_param_files = [
        str(base_params),
        mode_yaml_path,
    ]
    if params_overlay:
        overlay_path = Path(params_overlay).expanduser()
        if not overlay_path.is_file():
            raise FileNotFoundError(f'params_overlay not found: {overlay_path}')
        ee_param_files.append(str(overlay_path.resolve()))
        print(f'[vicon_hw] params_overlay={overlay_path.resolve()}')

    ee_stabilization = Node(
        package='arm_ee_stabilization_control',
        executable='ee_stabilization',
        name='ee_stabilization',
        output='screen',
        parameters=[
            *ee_param_files,
            {
                'urdf_path': str(ik_urdf),
                'base_source': 'tf',
                'hardware_mode': True,
                'stabilization_mode': selected,
            },
        ],
    )

    # Timeline
    t_arm_ready = 1.5
    if home_before:
        home = ExecuteProcess(
            cmd=[
                'python3', home_py,
                '--duration', str(home_duration),
                '--settle', str(home_settle),
                '--rate', '50',
            ],
            output='screen',
            name='move_to_home',
        )
        actions.append(TimerAction(period=t_arm_ready, actions=[home]))
        t_after_home = t_arm_ready + home_duration + home_settle + 0.5
    else:
        t_after_home = t_arm_ready

    # Bridge immediately after home (starts latch); controller after partial latch
    actions.append(TimerAction(period=t_after_home, actions=[bridge]))
    t_ctrl = t_after_home + max(0.5, latch_delay * 0.5)
    actions.append(TimerAction(period=t_ctrl, actions=[ee_stabilization]))

    print(
        f'[vicon_hw] home_before={home_before} '
        f't_home={t_arm_ready:.1f}s t_bridge={t_after_home:.1f}s t_ctrl={t_ctrl:.1f}s'
    )
    return actions


def generate_launch_description():
    default_vicon = os.path.expanduser('~/zihan_ws/vicon_perception/src')
    return LaunchDescription([
        DeclareLaunchArgument('stabilization_mode', default_value='A'),
        DeclareLaunchArgument('arm_type', default_value='a_l1'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyTHS3'),
        DeclareLaunchArgument('max_velocity', default_value='0.25'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('pose_topic', default_value='/vrpn/pregme/pose'),
        DeclareLaunchArgument('latch_delay', default_value='2.0'),
        DeclareLaunchArgument('mount_base_offset_z', default_value='0.02'),
        DeclareLaunchArgument(
            'start_vrpn', default_value='True',
            description=(
                'Launch vrpn_listener from vicon_ws. Default True because '
                'run_hw.sh claim_arm_serial often stops robot.service and its VRPN.'
            ),
        ),
        DeclareLaunchArgument('vicon_ws', default_value=default_vicon),
        DeclareLaunchArgument(
            'home_before_stabilize', default_value='True',
            description='Move arm to q_home before starting bridge/controller'),
        DeclareLaunchArgument('home_duration', default_value='6.0'),
        DeclareLaunchArgument('home_settle', default_value='0.8'),
        DeclareLaunchArgument(
            'params_overlay', default_value='',
            description=(
                'Optional YAML overlay merged last into ee_stabilization '
                '(Mode C auto-tune writes trial overlays here).'
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
