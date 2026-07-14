"""Launch EE stabilization on real hardware (student_arm + ee_stabilization).

Modes (``stabilization_mode`` launch arg):
  A - IK + MIT position tracking (recommended first on hardware)
  B - IK + computed-torque feedforward via MIT current
  C - task-space OSC torque feedforward via MIT current
  D - sat velocity planner + OSC + ESO (Wang 2024 inspired)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    selected = LaunchConfiguration('stabilization_mode').perform(context).upper()
    control_share = Path(get_package_share_directory('arm_ee_stabilization_control'))
    desc_share = Path(get_package_share_directory('arm_ee_stabilization_description'))
    manipulator_share = Path(get_package_share_directory('manipulator'))

    mode_files = {
        'A': control_share / 'config' / 'stabilization_hw_mode_a.yaml',
        'B': control_share / 'config' / 'stabilization_hw_mode_b.yaml',
        'C': control_share / 'config' / 'stabilization_hw_mode_c.yaml',
        'D': control_share / 'config' / 'stabilization_hw_mode_d.yaml',
    }
    mode_yaml_path = str(mode_files.get(selected, mode_files['A']))
    base_params = control_share / 'config' / 'stabilization.yaml'
    ik_urdf = desc_share / 'urdf' / 'single_arm.urdf'
    student_cfg = manipulator_share / 'stabilization_hw_student_arm.yaml'
    rviz_config = desc_share / 'rviz' / 'stabilization.rviz'

    arm_type = LaunchConfiguration('arm_type').perform(context)
    port_name = LaunchConfiguration('port_name').perform(context)
    max_velocity = float(LaunchConfiguration('max_velocity').perform(context))
    base_source = LaunchConfiguration('base_source').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')

    student_arm = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            str(student_cfg),
            {
                'arm_type': arm_type,
                'port_name': port_name,
                'max_velocity': max_velocity,
            },
        ],
    )

    ee_stabilization = Node(
        package='arm_ee_stabilization_control',
        executable='ee_stabilization',
        name='ee_stabilization',
        output='screen',
        parameters=[
            str(base_params),
            mode_yaml_path,
            {
                'urdf_path': str(ik_urdf),
                'base_source': base_source,
                'hardware_mode': True,
            },
        ],
    )

    urdf_path = os.path.join(str(manipulator_share), 'arm.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    nodes = [student_arm, rsp, ee_stabilization]
    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stabilization_mode', default_value='A',
            description='A=IK position, B=IK+CTC ff, C=OSC ff, D=sat+OSC+ESO'),
        DeclareLaunchArgument(
            'arm_type', default_value='a_l1',
            description='sim (bench test) or a_l1 (real hardware)'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyTHS3'),
        DeclareLaunchArgument('max_velocity', default_value='0.35'),
        DeclareLaunchArgument(
            'base_source', default_value='simulated',
            description='simulated | static | tf'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        OpaqueFunction(function=_setup),
    ])
