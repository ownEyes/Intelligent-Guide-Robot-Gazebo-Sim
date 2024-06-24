from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_launch_file_arg = DeclareLaunchArgument(
        'navigation_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation_simulation'),
            'launch',
            'slam_navigation.launch.py'
        ]),
        description='Full path to the navigation launch file.'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('navigation_simulation'),
            'rviz2',
            'view_navigation.rviz'
        ])],
        output='screen'
    )

    # Including the navigation launch file with a delay
    navigation_launch_include = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    LaunchConfiguration('navigation_launch_file')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    return LaunchDescription([
        rviz_node,
        navigation_launch_file_arg,
        navigation_launch_include,  # This now includes the AMCL launch file after a delay
    ])
