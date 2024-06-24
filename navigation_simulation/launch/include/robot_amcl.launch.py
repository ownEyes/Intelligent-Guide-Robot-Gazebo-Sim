from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('navigation_simulation'), 'param', 'amcl_params.yaml'
    ])

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            # Add or remove nodes as per your setup
            'node_names': ['amcl'],
            'bond_timeout': 10.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='The scan topic.'
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('/scan', LaunchConfiguration('scan_topic')),
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ],
        ),
        amcl_lifecycle_manager
    ])
