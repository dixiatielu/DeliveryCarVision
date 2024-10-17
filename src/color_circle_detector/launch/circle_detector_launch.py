from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('color_circle_detector'),
        'config',
        'circle_detector_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='color_circle_detector',
            executable='color_circle_detector',
            name='color_circle_detector',
            parameters=[config_file_path]
        )
    ])
