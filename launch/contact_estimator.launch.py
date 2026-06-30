from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('legged_state_estimator')

    default_urdf = PathJoinSubstitution([
        pkg_share, 'description', 'urdf', 'g1_29dof_rev_1_0.urdf'
    ])
    default_model = PathJoinSubstitution([
        pkg_share, 'model', 'results_h1_best_val_loss.pt'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='Absolute path to the robot URDF',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Absolute path to the trained .pt checkpoint',
        ),
        DeclareLaunchArgument('window_size',       default_value='150'),
        DeclareLaunchArgument('time_slop_ns',      default_value='50000000'),
        DeclareLaunchArgument('topic_imu',         default_value='/imu'),
        DeclareLaunchArgument('topic_joint',       default_value='/joint_states'),
        DeclareLaunchArgument('topic_label',       default_value='/contact_estimator/label'),
        DeclareLaunchArgument('topic_left',        default_value='/contact_estimator/left_contact'),
        DeclareLaunchArgument('topic_right',       default_value='/contact_estimator/right_contact'),
        Node(
            package='legged_state_estimator',
            executable='contact_estimator_node',
            name='contact_estimator',
            output='screen',
            parameters=[{
                'model_path':             LaunchConfiguration('model_path'),
                'urdf_path':              LaunchConfiguration('urdf_path'),
                'window_size':            LaunchConfiguration('window_size'),
                'time_slop_ns':           LaunchConfiguration('time_slop_ns'),
                'topic.imu':              LaunchConfiguration('topic_imu'),
                'topic.joint_states':     LaunchConfiguration('topic_joint'),
                'topic.contact_label':    LaunchConfiguration('topic_label'),
                'topic.left_contact':     LaunchConfiguration('topic_left'),
                'topic.right_contact':    LaunchConfiguration('topic_right'),
            }],
        ),
    ])
