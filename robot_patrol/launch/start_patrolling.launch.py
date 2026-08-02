from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    patrol_node = Node(
        package='robot_patrol',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
    )

    return LaunchDescription([
        patrol_node
    ])