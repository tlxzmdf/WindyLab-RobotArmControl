from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(context)
    urdf_path = os.path.join(pkg_share, urdf_file)
    rviz_config_path = os.path.join(pkg_share, 'student_arm.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            {'arm_type': LaunchConfiguration('arm_type').perform(context)},
            {'arm_version': 'gamma'},
            {'port_name': LaunchConfiguration('port_name').perform(context)},
            {'motor_config_path': motor_config_path},
            {'arm_config_path': arm_config_path},
            {'max_velocity': float(LaunchConfiguration('max_velocity').perform(context))},
            {'kinematic_mode': LaunchConfiguration('kinematic_mode').perform(context).lower() in ('true', '1', 'yes')},
            {'command_timeout_sec': float(
                LaunchConfiguration('command_timeout_sec').perform(context))},
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', joint_states_topic)],
    )

    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')
    nodes = [student_arm_node, robot_state_publisher_node]
    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='sim',
            description='Arm type: sim (virtual) or a_l1 (real hardware)',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyTHS3',
            description='Serial port for real hardware (ignored in sim mode)',
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.5',
            description='Max joint velocity (rad/s) safety limit for students',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Launch RViz for visualization',
        ),
        DeclareLaunchArgument(
            'kinematic_mode',
            default_value='False',
            description='Instant joint tracking (simulation / EE stabilization demo)',
        ),
        DeclareLaunchArgument(
            'command_timeout_sec',
            default_value='1.0',
            description='Command timeout before holding position (seconds)',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='arm.urdf',
            description='URDF filename under package share (e.g. arm_stabilization.urdf)',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='joint_states',
            description='Topic remapped to robot_state_publisher joint_states input',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
