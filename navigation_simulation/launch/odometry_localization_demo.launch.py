from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 地图文件路径参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/slam_sim_demo/maps/Software_Museum.yaml',
        description='Path to the map file.'
    )

    # map_server 节点配置
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}],
        output='screen'
    )

    # 静态TF广播器配置
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 移动基座配置
    # 假定navigation_sim_demo已经适配了ROS2，其启动文件也已经是Python格式
    move_base_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindExecutable(name='navigation_sim_demo'),
                'launch',
                'include',
                'move_base.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        static_tf_pub_node,
        move_base_include
    ])
