from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义地图文件路径的参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([FindPackageShare('slam_sim_demo'), 'maps', 'Software_Museum.yaml']),
        description='Path to the map file.'
    )

    # 定义初始位置参数
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='5.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_a_arg = DeclareLaunchArgument('initial_pose_a', default_value='-2.0')

    # 地图服务节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}],
        output='screen'
    )

    # 包含AMCL定位的配置
    custom_amcl_launch_file = LaunchConfiguration('custom_amcl_launch_file', default=PathJoinSubstitution([
        FindPackageShare('navigation_sim_demo'), 'launch', 'include', 'robot_amcl.launch.py'
    ]))

    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_amcl_launch_file),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_a': LaunchConfiguration('initial_pose_a'),
        }.items(),
    )

    # 包含移动基座的配置
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
        map_server_node,
        amcl_include,
        move_base_include
    ])

