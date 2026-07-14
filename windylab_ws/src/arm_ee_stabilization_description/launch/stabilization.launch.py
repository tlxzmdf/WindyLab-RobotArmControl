from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("arm_ee_stabilization_description"))
    control_share = Path(get_package_share_directory("arm_ee_stabilization_control"))
    display_urdf = package_share / "urdf" / "arm_on_drone.urdf"
    ik_urdf = package_share / "urdf" / "single_arm.urdf"
    rviz_config = package_share / "rviz" / "stabilization.rviz"
    control_params = control_share / "config" / "stabilization.yaml"
    robot_description = display_urdf.read_text()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 500.0,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="arm_ee_stabilization_control",
                executable="ee_stabilization",
                name="ee_stabilization",
                output="screen",
                parameters=[
                    str(control_params),
                    {"urdf_path": str(ik_urdf)},
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", str(rviz_config)],
                        output="screen",
                    )
                ],
            ),
        ]
    )
