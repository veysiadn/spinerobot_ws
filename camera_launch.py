from launch import LaunchDescription
## For custom node launchs
from launch_ros.actions import Node

import ament_index_python
import launch
import launch_ros
import os


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(package='custom_image_view', node_executable='custom_image_view', output='screen',prefix = 'taskset -c 2'),
        Node(package='usb_camera_driver', node_executable='usb_camera_driver_node', output='screen',prefix = 'taskset -c 2')
        ])