import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("legged_state_estimator")

    urdf_path = LaunchConfiguration("urdf_path")
    estimator_type = LaunchConfiguration("estimator_type")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Logging level: DEBUG | INFO | WARN | ERROR",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=os.path.join(
                pkg_share, "description", "urdf", "g1_29dof_rev_1_0.urdf"
            ),
            description="Path to robot URDF file",
        ),
        DeclareLaunchArgument(
            "estimator_type",
            default_value="fixed_lag_single_bias",
            description="Estimator variant: invariant_ekf | invariant_graph | "
                        "fixed_lag_single_bias | fixed_lag_combined_bias",
        ),
        DeclareLaunchArgument(
            "disable_contact",
            default_value="false",
            description="Disable contact updates (IMU-only dead reckoning for debug)",
        ),
        DeclareLaunchArgument(
            "zero_accel_debug",
            default_value="false",
            description="Replace IMU accel with gravity-only (debug: isolate accel drift)",
        ),
        DeclareLaunchArgument(
            "zero_gyro_debug",
            default_value="false",
            description="Replace IMU gyro with zero (debug: isolate gyro contribution)",
        ),
        Node(
            package="legged_state_estimator",
            executable="humanoid_state_estimator",
            name="humanoid_state_estimator",
            output="screen",
            arguments=["--ros-args", "--log-level",
                       LaunchConfiguration("log_level")],
            parameters=[{
                "urdf_path": urdf_path,
                "estimator_type": estimator_type,
                "contact_force_threshold": 10.0,
                "initial_height": 0.787,
                "sigma_gyro": 8e-4,
                "sigma_acc": 2e-2,
                "lag_seconds": 1.0,
                "world_frame": "odom",
                "base_frame": "base_link",
                "disable_contact": LaunchConfiguration("disable_contact"),
                "zero_accel_debug": LaunchConfiguration("zero_accel_debug"),
                "zero_gyro_debug": LaunchConfiguration("zero_gyro_debug"),
            }],
        ),
    ])
