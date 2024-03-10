#!/usr/bin/env python3

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    publish_frequency = LaunchConfiguration('publish_frequency', default='20.0')
    # Path to your .xacro file
    xacro_file_path = os.path.join(
        get_package_share_directory('robot_simulation'),
        'urdf',
        'robot.xacro'
    )


    print('xacro_file_path : {}'.format(xacro_file_path))

    # Process .xacro file to generate URDF
    robot_desc = xacro.process_file(xacro_file_path).toprettyxml(indent=' ')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='20.0',
            description='Frequency at which to publish the robot state'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='/xbot',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency':publish_frequency,
                'robot_description': robot_desc
            }],
        ),
    ])