from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('qr_code_detector'),
        'config',
        'qr_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='qr_code_detector',
            executable='qr_code_detector',
            name='qr_code_detector',
            parameters=[config_file_path]
        )
    ])
