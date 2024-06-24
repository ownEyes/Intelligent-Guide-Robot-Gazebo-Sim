from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定义参数
    params_file = PathJoinSubstitution([
        FindPackageShare('navigation_simulation'), 'param', 'nav2_params.yaml'
    ])

    # Navigation2启动文件的包含
    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(
                    'nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': params_file,
            'autostart': 'True',
            'use_sim_time': 'True',
        }.items()
    )

    return LaunchDescription([
        nav2_launch_file
        # 你可以在这里添加更多节点或包含其他启动文件
    ])
