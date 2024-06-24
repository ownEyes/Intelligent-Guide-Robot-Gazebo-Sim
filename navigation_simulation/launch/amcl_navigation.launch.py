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
            [FindPackageShare('navigation_simulation'), 'maps', 'museum.yaml']),
        description='Path to the map file.'
    )

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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sensor_timeout': 0.1,  # seconds
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',  # Fixed frame
            'odom_frame': 'odom',  # Odometry frame
            'base_link_frame': 'base_footprint',  # Robot base frame
            # Frame to track (set to 'odom' if not using map->odom transform)
            'world_frame': 'odom',
            'frequency': 30.0,  # Frequency of filter updates
            'odom0': 'odom',  # Topic for odometry input
            'imu0': 'imu',  # Topic for IMU input
            'odom0_config': [True, True, False,
                             False, False, True,
                             True, False, False,
                             False, False, True,
                             False, False, False],
            'imu0_config': [
                False, False, False,   # x, y, z positions
                False, False, False,   # x, y, z velocities
                True, True, True,      # roll, pitch, yaw orientations
                False, False, True,    # x, y, z angular velocities
                True, True, True       # x, y, z linear accelerations
            ],
        }],
        remappings=[('/odometry/filtered', 'odom')]
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
                    'laser_scan_topic': '/scan',
                    'odom_topic': '/odom_rf2o',
                    'publish_tf': False,
                    'base_frame_id': 'base_footprint',
                    'odom_frame_id': 'odom',
                    'init_pose_from_topic': '',
                    'freq': 10.0}],
    )

    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',  # Changed from ekf_node to ukf_node
        name='ukf_filter_node',  # Updated name for clarity
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sensor_timeout': 1.0,  # seconds
            # 'print_diagnostics': True,
            # 'permit_corrected_publication': True,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',  # Fixed frame
            'odom_frame': 'odom',  # Odometry frame
            'base_link_frame': 'base_footprint',  # Robot base frame
            # Frame to track (set to 'odom' if not using map->odom transform)
            'world_frame': 'odom',
            'frequency': 50.0,  # Frequency of filter updates
            # Inputs: odom from RF2O and another raw odometry source
            'odom0': 'odom_rf2o',
            'odom1': 'odom_raw',

            # Configure each input
            'odom0_config': [True, True, False,
                             False, False, False,
                             False, False, True,
                             False, False, False,
                             False, False, False],

            'odom1_config': [True, True, False,
                             False, False, False,
                             False, False, True,
                             False, False, False,
                             False, False, False],

            # 'odom0_queue_size': 1,  # 10hz 0.1
            # 'odom1_queue_size': 2,  # 20hz 0.05

            'imu0': 'imu',  # Topic for IMU input
            'imu0_config': [
                False, False, False,   # x, y, z positions
                False, False, False,   # x, y, z velocities
                False, False, False,      # roll, pitch, yaw orientations
                False, False, True,    # x, y, z angular velocities
                True, True, False       # x, y, z linear accelerations
            ],

            # 'imu0_queue_size': 10,  # 100hz 0.001

            'use_control': True,
            'control_config': [
                True, False, False,   # x, y, z positions
                False, False, True,
            ],
            'control_timeout': 1.0,
            # 'dynamic_process_noise_covariance': True,
            # 'odom0_relative': True,
            # 'odom0_differential': True,
            # 'odom1_relative': True,
            # 'odom1_differential': True,
            'imu0_relative': True,
            'alpha': 0.001,
            'kappa': 0.0,
            'beta': 2.0,
            'acceleration_limits': [2.5, 0.0, 0.0, 0.0, 0.0, 3.2],
            'deceleration_limits': [2.5, 0.0, 0.0, 0.0, 0.0, 3.2],
        }],
        remappings=[('/odometry/filtered', 'odom')]
    )

    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('custom_amcl_launch_file')),
    )

    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(
                'navigation_simulation'), 'launch', 'include', 'navigation_bringup.launch.py'
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
            'node_names': ['controller_server', 'bt_navigator', 'waypoint_follower', 'planner_server', 'velocity_smoother'],
            'bond_timeout': 10.0
        }]
    )

    return LaunchDescription([
        rf2o_node,
        ukf_node,
        set_env_vars_resources,
        map_file_arg,
        map_server_node,
        map_server_lifecycle_manager,
        custom_amcl_launch_file_arg,
        amcl_include,
        nav2_bringup_include,
        # ekf_node,
        navigation_lifecycle_manager,
    ])
