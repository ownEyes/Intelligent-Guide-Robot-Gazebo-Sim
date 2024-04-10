from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',  # 注意，ROS2中rviz的包名为rviz2
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/navigation_simulation/rviz2/navigation.rviz'],
            # 如果navigation.rviz文件位于某个包中，可以使用launch_ros.substitutions.FindPackageShare来找到它
            # 例如: launch_ros.substitutions.FindPackageShare('navigation_sim_demo') + '/rviz/navigation.rviz'
            output='screen'
        )
    ])
