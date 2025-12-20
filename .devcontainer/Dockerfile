# 基础镜像（ROS Humble 桌面完整版）
FROM osrf/ros:humble-desktop-full

# 设置构建参数，避免交互式询问
ARG DEBIAN_FRONTEND=noninteractive
# 用户参数
ARG USERNAME=yutou
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 1. 安装系统依赖包和Ignition Gazebo
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        git \
        ros-humble-nav2-bringup \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-robot-state-publisher \
        fish \
        gedit \
        ros-humble-joy \
        ros-humble-rqt-tf-tree \
        ros-humble-foxglove-bridge \
        nautilus \
        curl \
        sudo \
        libpcap-dev \
        python3-pip \
        lsb-release \
        gnupg && \
    # 安装Ignition Gazebo Fortress
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y ignition-fortress && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -a -G dialout $USERNAME

# 3. 设置系统级环境变量
ENV ROS_LOCALHOST_ONLY=2
ENV IGN_GAZEBO_RESOURCE_PATH=/home/yutou/ros2_ws/rmoss_gz_resources/resource/models

# 4. 切换到非root用户
USER $USERNAME
WORKDIR /home/$USERNAME

# 5. 设置用户环境变量
ENV ROS2_DISTRO=humble
ENV ROS2_SETUP_BASH="/opt/ros/humble/setup.bash"

# 6. 在.bashrc中再次设置环境变量（确保生效）
RUN echo "source $ROS2_SETUP_BASH" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=2" >> ~/.bashrc && \
    echo "export IGN_GAZEBO_RESOURCE_PATH=/home/yutou/ros2_ws/rmoss_gz_resources/resource/models" >> ~/.bashrc

# 7. 使用登录shell确保.bashrc被加载
CMD ["bash", "-l"]