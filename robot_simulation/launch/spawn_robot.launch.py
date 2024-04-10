#!/usr/bin/env python3

import os
# import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    xacro_file_path = os.path.join(
        get_package_share_directory('robot_simulation'),
        'urdf',
        'robot.xacro'
    )
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_yaw_position_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Specify namespace of the robot')

    # Convert xacro to URDF
    robot_description_content = Command([
        'xacro ', xacro_file_path,
        ' model:=', 'xbot-u'
    ])

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'xbot-u',
            '-string', robot_description_content,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory('robot_simulation'),
        'param',
        'xbot-u_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    controller_params_file = os.path.join(
        get_package_share_directory('robot_simulation'),
        'param',
        'xbot-u_control.yaml'
    )

    with open(controller_params_file, 'r') as f:
        controller_params = yaml.safe_load(f)

    start_spawn_controllers_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='/xbot',
        arguments=[
            '--controller-manager',
            '/xbot/controller_manager',
            'joint_state_controller',
            'yaw_platform_position_controller',
            'pitch_platform_position_controller',
        ],
        output='screen',
        parameters=[controller_params, {'use_sim_time': True}]
    )

    vel_mux_params_file = os.path.join(
        get_package_share_directory('robot_simulation'),
        'param',
        'mux.yaml'
    )

    with open(vel_mux_params_file, 'r') as f:
        vel_mux_params = yaml.safe_load(f)['cmd_vel_mux']['ros__parameters']

    start_cmd_vel_mux_node_cmd = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        output='both',
        parameters=[vel_mux_params, {'use_sim_time': True}]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_yaw_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(start_spawn_controllers_cmd)
    ld.add_action(start_cmd_vel_mux_node_cmd)

    return ld
