# 基于轻量级的ROS Humble基础镜像
FROM ros:humble-ros-base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# 安装必要依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    libboost-all-dev \
    supervisor

# 复制你的ROS包到容器内
COPY . /ros_ws
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# 构建ROS工作空间
WORKDIR /ros_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# 设置supervisord为入口点
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
