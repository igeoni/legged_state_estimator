import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    pkg_share = get_package_share_directory("legged_state_estimator")

    urdf_path = LaunchConfiguration("urdf_path")
    estimator_type = LaunchConfiguration("estimator_type")

    # Point directly at the source tree so edits in RViz persist across builds
    rviz_config_path = os.path.join(
        _SCRIPT_DIR, "..", "rviz", "unitree_g1.rviz"
    )

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
            default_value="invariant_graph",
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
            parameters=[
                os.path.join(pkg_share, "config", "humanoid_estimator.yaml"),
                {
                    "urdf_path": urdf_path,
                    "estimator_type": estimator_type,
                    "disable_contact": LaunchConfiguration("disable_contact"),
                    "zero_accel_debug": LaunchConfiguration("zero_accel_debug"),
                    "zero_gyro_debug": LaunchConfiguration("zero_gyro_debug"),
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', rviz_config_path],
            output={'stdout': 'screen', 'stderr': 'log'},
        )
    ])
