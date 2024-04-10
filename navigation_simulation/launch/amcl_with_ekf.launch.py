from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([FindPackageShare('slam_sim_demo'), 'maps', 'Software_Museum.yaml']),
        description='Path to the map file.'
    )

    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='5.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_a_arg = DeclareLaunchArgument('initial_pose_a', default_value='-2.0')
    custom_amcl_launch_file_arg = DeclareLaunchArgument(
        'custom_amcl_launch_file',
        default_value=PathJoinSubstitution([FindPackageShare('navigation_sim_demo'), 'launch', 'include', 'robot_amcl.launch.py'])
    )

    # 地图服务
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}],
        output='screen'
    )

    # AMCL配置
    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('custom_amcl_launch_file')),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_a': LaunchConfiguration('initial_pose_a'),
        }.items(),
    )

    # EKF配置
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sensor_timeout': 1.0,
            'two_d_mode': True,
            'map_frame': 'map',  
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            'frequency': 30.0,
            'odom0': 'odom',
            'imu0': 'imu',
            'odom0_config': [True, True, False,
                             False, False, True,
                             True, True, True,
                             False, False, True,
                             False, False, False],
        }],
        remappings=[('imu_data', 'imu')]
    )

    # 移动基座配置
    move_base_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('navigation_sim_demo'),
                'launch',
                'include',
                'move_base.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        map_file_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_a_arg,
        custom_amcl_launch_file_arg,
        map_server_node,
        amcl_include,
        ekf_node,
        move_base_include,
    ])

