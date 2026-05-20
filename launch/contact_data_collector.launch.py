from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("legged_state_estimator")

    urdf_path = PathJoinSubstitution([
        pkg_share, "description", "urdf", "g1_29dof_with_hand.urdf"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=urdf_path,
            description="Absolute path to the robot URDF file",
        ),
        DeclareLaunchArgument("output_dir",          default_value="./dataset"),
        DeclareLaunchArgument("max_samples",         default_value="10000"),
        DeclareLaunchArgument("time_slop_ns",        default_value="50000000"),
        DeclareLaunchArgument("contact_threshold_n", default_value="10.0"),
        Node(
            package="legged_state_estimator",
            executable="contact_data_collector",
            name="contact_data_collector",
            output="screen",
            parameters=[{
                "urdf_path":            LaunchConfiguration("urdf_path"),
                "output_dir":           LaunchConfiguration("output_dir"),
                "max_samples":          LaunchConfiguration("max_samples"),
                "time_slop_ns":         LaunchConfiguration("time_slop_ns"),
                "contact_threshold_n":  LaunchConfiguration("contact_threshold_n"),
            }],
        ),
    ])
