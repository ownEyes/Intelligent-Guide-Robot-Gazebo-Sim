#!/usr/bin/env python3

import os
# import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Get the urdf file
    xacro_file_path = os.path.join(
        get_package_share_directory('robot_simulation'),
        'urdf',
        'robot.xacro'
    )
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='5.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='-2.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='5.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_yaw_position_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='-2.0',
        description='Specify namespace of the robot')

    # Convert xacro to URDF
    robot_description_content = Command([
        'xacro ', xacro_file_path,
        ' model:=', 'xbot-u'
    ])
    # sdf_path = os.path.join(
    #     get_package_share_directory('robot_simulation'),
    #     'urdf',
    #     'model',
    #     'xbot-u.sdf'
    # )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'xbot-u',
            '-string', robot_description_content,
            # '-file', sdf_path,
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
        arguments=[
            # '/camera/depth',
            '/camera/depth/image',  # RGB image
            '/camera/depth/depth_image',
            # '/camera/rgb/camera_info',
            # '/camera/depth/image_raw',  # Depth image
            # '/camera/depth/points',  # Depth image
        ],
        output='screen',
    )

    # controller_params_file = os.path.join(
    #     get_package_share_directory('robot_simulation'),
    #     'param',
    #     'xbot-u_control.yaml'
    # )

    # with open(controller_params_file, 'r') as f:
    #     controller_params = yaml.safe_load(f)

    # start_spawn_controllers_cmd = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ('/controller_manager/robot_description', '/robot_description')]
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'effort_controller'],
        output='screen'
    )

    set_contoller_manager_use_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/controller_manager', 'use_sim_time', 'true'],
        output='screen')

    load_joint_state_broadcaster_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_ros_spawner_cmd,
            # on_exit=[set_contoller_manager_use_sim_time,
            #          load_joint_state_broadcaster],
            on_exit=[set_contoller_manager_use_sim_time],
        )
    )

    load_effort_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_ros_spawner_cmd,
            on_exit=[load_effort_controller],
        )
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

    ld.add_action(load_joint_state_broadcaster_handler)
    # ld.add_action(load_effort_controller_handler)
    # ld.add_action(start_spawn_controllers_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)

    # ld.add_action(start_cmd_vel_mux_node_cmd)

    return ld
