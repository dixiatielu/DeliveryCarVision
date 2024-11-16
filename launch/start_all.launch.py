import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动上方相机节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[os.path.join(
                os.path.expanduser('~'),
                'DeliveryCarVision',
                'config',
                'cam_params.yaml'
            )],
            respawn=True
        ),
        
        # 启动yolo节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                os.path.expanduser('~'),
                'DeliveryCarVision',
                'src',
                'yolo_ros',
                'yolo_bringup',
                'launch',
                'yolo.launch.py'
            )]),
            launch_arguments={
                'imgsz_height': '640',
                'imgsz_width': '640',
                'device': 'cpu',
                'half': 'True',
                'model': 'ccv2-y11n_ncnn_model',
                'use_tracking': 'False',
                'input_image_topic': '/image_raw',
                'use_debug': 'True'
            }.items(),
        ),
        
        # 启动串口节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                os.path.expanduser('~'),
                'DeliveryCarVision',
                'src',
                'transport_drivers',
                'serial_driver',
                'launch',
                'serial_driver_bridge_node.launch.py'
            )]),
            launch_arguments={
                'params_file': os.path.join(
                    os.path.expanduser('~'),
                    'DeliveryCarVision',
                    'config',
                    'serial.params.yaml'
                )
            }.items(),
        ),
        
        # 启动圆环检测节点
        Node(
            package='circle_detector',
            executable='circle_detector',
            name='circle_detector',
            output='screen',
            respawn=True
        ),

        # 启动QR Code检测节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                os.path.expanduser('~'),
                'DeliveryCarVision',
                'src',
                'qr_code_detector',
                'launch',
                'qr_detector_launch.py'
            )]),
        )
    ])