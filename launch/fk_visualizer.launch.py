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
        # Publishes TF from the same URDF + /joint_states.
        # If FK markers match this TF => Pinocchio is correct, Isaac Sim uses a different model.
        # If FK markers don't match => bug in FK code.
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     output="screen",
        #     parameters=[{"robot_description": robot_description}],
        # ),
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
