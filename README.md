# 工创物流-视觉

## 运行环境：
- Raspberry Pi 5
- Ubuntu 24.04
- Python 3.12
- ROS2-jazzy
- OpenCV

## 重要依赖包
- [yolo_ros](https://github.com/mgonzs13/yolo_ros) 已经子模块包含在本仓库中
- [usb_cam](https://github.com/ros-drivers/usb_cam) 需要通过apt安装 `sudo apt install ros-jazzy-usb-cam`

其他的参照`package.xml`文件，可使用`rosdep`安装。

## 编译
```shell
colcon build
```
## 运行节点

### 一键启动
```shell
source install/setup.bash
ros2 launch launch/start_all.launch.py
```

### 手动启动

Refer：
- [usb_cam Running](https://github.com/ros-drivers/usb_cam?tab=readme-ov-file#running)
- [yolo_ros Parameters](https://github.com/mgonzs13/yolo_ros?tab=readme-ov-file#parameters)

```shell
cd ~/DeliveryCarVision
source install/setup.bash
# 启动上方相机节点
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./config/cam_params.yaml 
# 启动yolo节点
ros2 launch yolo_bringup yolo.launch.py imgsz_height:=640 imgsz_width:=640 device:=cpu half:=True model:=ccv2-y11n_ncnn_model use_tracking:=False input_image_topic:=/image_raw use_debug:=True
# 启动串口节点
ros2 launch serial_driver serial_driver_bridge_node.launch.py params_file:=config/serial.params.yaml
# 启动圆环检测节点
ros2 run circle_detector circle_detector
```