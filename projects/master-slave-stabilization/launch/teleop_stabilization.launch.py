"""真主臂 + 仿真从臂（末端自稳 + 主臂遥操作跟踪）。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _setup(context, *args, **kwargs):
    project_root = _project_root()
    teleop_config_name = LaunchConfiguration('teleop_config').perform(context)
    teleop_params = str(project_root / 'config' / teleop_config_name)

    manipulator_share = Path(get_package_share_directory('manipulator'))
    desc_share = Path(get_package_share_directory('arm_ee_stabilization_description'))
    control_share = Path(get_package_share_directory('arm_ee_stabilization_control'))

    motor_config_path = PathJoinSubstitution(
        [FindPackageShare('manipulator'), 'motor_config.yaml'])
    arm_config_path = PathJoinSubstitution(
        [FindPackageShare('manipulator'), 'arm_config.yaml'])
    master_urdf = str(project_root / 'urdf' / 'arm_link7_zero_mass.urdf')

    display_urdf = desc_share / 'urdf' / 'arm_on_drone.urdf'
    ik_urdf = desc_share / 'urdf' / 'single_arm.urdf'
    base_params = control_share / 'config' / 'stabilization.yaml'
    rviz_config = desc_share / 'rviz' / 'stabilization.rviz'

    master_arm_type = LaunchConfiguration('master_arm_type').perform(context)
    master_mode = LaunchConfiguration('master_mode').perform(context)
    port_name = LaunchConfiguration('port_name').perform(context)
    base_source_arg = LaunchConfiguration('base_source').perform(context)
    disturbance_profile = LaunchConfiguration('disturbance_profile').perform(context)
    max_velocity = float(LaunchConfiguration('max_velocity').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')

    scripted_profiles = {'sway', 'pitch_step', 'idle'}
    if disturbance_profile in scripted_profiles:
        base_source_override = 'external'
    elif base_source_arg:
        base_source_override = base_source_arg
    else:
        base_source_override = None

    master_joint_topic = '/master/joint_states'
    nodes = []

    if master_mode == 'backdrive':
        nodes.append(Node(
            package='manipulator',
            executable='master_arm_node',
            name='master_arm_node',
            namespace='master',
            output='screen',
            parameters=[
                {'arm_type': master_arm_type},
                {'arm_version': 'gamma'},
                {'auto_reset': False},
                {'port_name': port_name},
                {'motor_config_path': motor_config_path},
                {'arm_config_path': arm_config_path},
                {'MAX_TORQUE': 3.0},
                {'GRAVITY': 9.81},
                {'debug_info': False},
                {'FORCE_FEEDBACK_THRESHOLD': 0.5},
                {'FORCE_FEEDBACK_GAIN': 0.0},
                {'publish_joint_state': True},
                {'publish_joint_feedback': False},
                {'urdf_path': master_urdf},
            ],
        ))
    elif master_mode == 'position':
        nodes.append(Node(
            package='manipulator',
            executable='student_arm_node',
            name='student_arm_node',
            output='screen',
            remappings=[('joint_states', '/master/joint_states')],
            parameters=[
                {'arm_type': master_arm_type},
                {'arm_version': 'gamma'},
                {'port_name': port_name},
                {'motor_config_path': motor_config_path},
                {'arm_config_path': arm_config_path},
                {'max_velocity': max_velocity},
                {'kinematic_mode': False},
                {'command_timeout_sec': 2.0},
            ],
        ))
    # scripted: 不启动主臂节点，由 master_motion_demo 等外部发布 /master/joint_states

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': display_urdf.read_text(),
                'publish_frequency': 500.0,
                'use_sim_time': False,
            }
        ],
    ))

    ee_params = {
        'urdf_path': str(ik_urdf),
        'master_urdf_path': master_urdf,
        'master_joint_topic': master_joint_topic,
    }
    if base_source_override is not None:
        ee_params['base_source'] = base_source_override

    nodes.append(Node(
        package='arm_ee_stabilization_control',
        executable='ee_stabilization',
        name='ee_stabilization',
        output='screen',
        parameters=[
            str(base_params),
            teleop_params,
            ee_params,
        ],
    ))

    if disturbance_profile in scripted_profiles:
        nodes.append(ExecuteProcess(
            cmd=[
                'python3',
                str(project_root / 'scripts' / 'mount_disturbance_profile.py'),
                '--ros-args',
                '-p', f'profile:={disturbance_profile}',
            ],
            output='screen',
        ))

    if use_rviz:
        nodes.append(
            TimerAction(
                period=2.0,
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
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'master_arm_type', default_value='a_l1',
            description='sim | a_l1（仅 backdrive/position 模式）'),
        DeclareLaunchArgument(
            'master_mode', default_value='backdrive',
            description='backdrive=零力重力补偿 | scripted=外部轨迹 | position=位置控制'),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyUSB0',
            description='主臂真机串口'),
        DeclareLaunchArgument('max_velocity', default_value='0.25'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument(
            'base_source', default_value='',
            description='覆盖 config 中 base_source；留空则用 YAML'),
        DeclareLaunchArgument(
            'teleop_config', default_value='teleop_stabilization.yaml',
            description='teleop 参数 YAML（位于 config/）'),
        DeclareLaunchArgument(
            'disturbance_profile', default_value='random',
            description='random=内置随机 | sway=连续晃动 | pitch_step=Pitch阶跃 | idle=水平（external 注入）'),
        OpaqueFunction(function=_setup),
    ])
