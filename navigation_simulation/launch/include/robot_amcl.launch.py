from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('navigation_simulation'), 'param', 'amsl_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_map_topic',
            default_value='true',
            description='Whether to subscribe to the map topic.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether to use Gazebo clock'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='The scan topic.'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose in X.'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose in Y.'
        ),
        DeclareLaunchArgument(
            'initial_pose_a',
            default_value='0.0',
            description='Initial pose orientation (yaw).'
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='The odometry frame id.'
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_footprint',
            description='The robot base frame id.'
        ),
        DeclareLaunchArgument(
            'global_frame_id',
            default_value='map',
            description='The global frame id.'
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('scan', LaunchConfiguration('scan_topic')),
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ],
        )
    ])
