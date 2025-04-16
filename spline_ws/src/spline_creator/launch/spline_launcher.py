#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for the spline_creator package.
    package_share = get_package_share_directory('spline_creator')

    # Path to your RViz config file (assuming it's located in the rviz folder).
    rviz_config_file = os.path.join(package_share, 'rviz', 'path_display.rviz')

    # Node for your spline_creator node.
    spline_node = Node(
        package='spline_creator',
        executable='spline_creator_node',
        name='spline_creator_node',
        output='screen'
    )

    # Node for launching RViz2 with your config.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        spline_node,
        rviz_node
    ])
