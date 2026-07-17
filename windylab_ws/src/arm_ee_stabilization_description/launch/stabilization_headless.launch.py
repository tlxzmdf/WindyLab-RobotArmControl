"""Headless launch for automated testing (no RViz)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("arm_ee_stabilization_description"))
    control_share = Path(get_package_share_directory("arm_ee_stabilization_control"))
    display_urdf = package_share / "urdf" / "arm_on_drone.urdf"
    ik_urdf = package_share / "urdf" / "single_arm.urdf"
    control_params = control_share / "config" / "stabilization.yaml"
    robot_description = display_urdf.read_text()

    use_ik_arg = DeclareLaunchArgument("use_ik_joint_control", default_value="true")
    kinematic_arg = DeclareLaunchArgument("kinematic_stabilization", default_value="true")
    mode_arg = DeclareLaunchArgument("stabilization_mode", default_value="")

    mode_d_params = control_share / "config" / "stabilization_mode_d.yaml"
    mode_e_params = control_share / "config" / "stabilization_mode_e.yaml"

    return LaunchDescription(
        [
            use_ik_arg,
            kinematic_arg,
            mode_arg,
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
                    str(mode_d_params),
                    str(mode_e_params),
                    {"urdf_path": str(ik_urdf)},
                    {"stabilization_mode": LaunchConfiguration("stabilization_mode")},
                    {"use_ik_joint_control": LaunchConfiguration("use_ik_joint_control")},
                    {"kinematic_stabilization": LaunchConfiguration("kinematic_stabilization")},
                ],
            ),
        ]
    )
