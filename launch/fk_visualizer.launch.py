from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("legged_state_estimator")

    urdf_path = PathJoinSubstitution([
        pkg_share, "description", "urdf", "g1_29dof_rev_1_0.urdf"
    ])

    robot_description = Command(["cat ", urdf_path])

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=urdf_path,
            description="Absolute path to the robot URDF file",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="pelvis",
            description="Root frame for FK marker visualization",
        ),
        Node(
            package="legged_state_estimator",
            executable="fk_visualizer",
            name="fk_visualizer",
            output="screen",
            parameters=[{
                "urdf_path": LaunchConfiguration("urdf_path"),
                "base_frame": LaunchConfiguration("base_frame"),
            }],
        ),
    ])
