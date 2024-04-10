import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定义参数
    # map_file_arg = DeclareLaunchArgument(
    #     'map_file',
    #     default_value=PathJoinSubstitution(
    #         [FindPackageShare('slam_simulation'), 'maps', 'Software_Museum.yaml']),
    #     description='Path to the map file.'
    # )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('robot_simulation'),
                     'models'))

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('navigation_simulation'), 'maps', 'Software_Museum.yaml']),
        description='Path to the map file.'
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='5.0')
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0')
    initial_pose_a_arg = DeclareLaunchArgument(
        'initial_pose_a', default_value='-2.0')

    custom_amcl_launch_file_arg = DeclareLaunchArgument(
        'custom_amcl_launch_file',
        default_value=PathJoinSubstitution([FindPackageShare(
            'navigation_simulation'), 'launch', 'include', 'robot_amcl.launch.py']),
    )

    # 地图服务节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file'),
                     'use_sim_time': True,
                     'topic_name': "map",
                     'frame_id': "map",
                     }],
        output='screen'
    )

    # Map server lifecycle manager node
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 10.0
        }]
    )

    # AMCL定位
    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('custom_amcl_launch_file')),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_a': LaunchConfiguration('initial_pose_a'),
        }.items(),
    )

    # 移动基座
    move_base_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(
                'navigation_simulation'), 'launch', 'include', 'move_base.launch.py'
        ])),
    )

    navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            # Add or remove nodes as per your setup
            'node_names': ['amcl', 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower'],
            'bond_timeout': 10.0
        }]
    )

    return LaunchDescription([
        set_env_vars_resources,
        map_file_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_a_arg,
        map_server_node,
        map_server_lifecycle_manager,
        custom_amcl_launch_file_arg,
        amcl_include,
        move_base_include,
        navigation_lifecycle_manager,
    ])
