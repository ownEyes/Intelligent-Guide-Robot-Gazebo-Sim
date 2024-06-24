from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set up the path to the navigation launch file
    nav2_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])

    # Set up the path to the SLAM Toolbox launch file
    slam_launch_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('slam_simulation'),
                'rviz2',
                'view_slam.rviz'
            ])],
            output='screen'
        ),
        # Include Navigation2 Launch File
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # Include SLAM Toolbox Launch File
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
