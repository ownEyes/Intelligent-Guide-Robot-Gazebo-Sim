import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    slam_launch_file_arg = DeclareLaunchArgument(
        'slam_launch_file',
        default_value=PathJoinSubstitution([FindPackageShare(
            'navigation_simulation'), 'launch', 'include', 'robot_slam.launch.py']),
    )
    # publish_initial_pose = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'topic', 'pub', '--rate', '1', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
    #         '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8415, w: 0.5403}}, covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}',
    #     ],
    #     output='screen',
    # )
    publish_initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8415, w: 0.5403}}, covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}',
            '--once'
        ],
        output='screen',
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
            'base_link_frame': 'base_link',  # Robot base frame
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
        # remappings=[('imu_data', 'imu')]
    )

    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',  # Changed from ekf_node to ukf_node
        name='ukf_filter_node',  # Updated name for clarity
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sensor_timeout': 0.1,  # seconds
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',  # Fixed frame
            'odom_frame': 'odom',  # Odometry frame
            'base_link_frame': 'base_link',  # Robot base frame
            # Frame to track (set to 'odom' if not using map->odom transform)
            'world_frame': 'odom',
            'frequency': 100.0,  # Frequency of filter updates
            'odom0': 'odom_raw',  # Topic for odometry input
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

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('slam_launch_file')),
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
            'node_names': ['controller_server', 'bt_navigator', 'waypoint_follower', 'planner_server'],
            'bond_timeout': 10.0
        }]
    )

    reset_map = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/slam_toolbox/reset',
             'slam_toolbox/srv/Reset', '{}'],
        output='screen'
    )

    return LaunchDescription([
        set_env_vars_resources,
        map_file_arg,
        map_server_node,
        map_server_lifecycle_manager,
        slam_launch_file_arg,
        slam_include,
        publish_initial_pose,
        nav2_bringup_include,
        # ekf_node,
        ukf_node,
        navigation_lifecycle_manager,
        reset_map,
    ])
