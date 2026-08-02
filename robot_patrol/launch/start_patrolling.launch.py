import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to a saved RViz config file inside this package (see notes below
    # on how to create/save one). If the file doesn't exist yet, RViz will
    # still launch fine with its default (blank) config.
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol.rviz'
    )

    patrol_node = Node(
        package='robot_patrol',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        patrol_node,
        rviz_node
    ])