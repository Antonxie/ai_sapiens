FROM ros:jazzy

WORKDIR /root

# onnxruntime 采用本地缓存（github release 下载在此网络不可达），
# 从 PyPI wheel + GitHub raw 头文件组装于 deps_onnxruntime/
COPY deps_onnxruntime/include/onnxruntime /usr/local/include/onnxruntime
COPY deps_onnxruntime/lib/. /usr/local/lib/

RUN ldconfig

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src

COPY . ${COLCON_WS}/src/ai_sapiens

ENV ROS_DISTRO=jazzy

# 依赖安装（无 rosdep：rosdistro 网络不可达，改为手动 apt 清单；
# ROS apt 源走阿里云，ubuntu 源走官方 archive.ubuntu.com）
RUN sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.aliyun.com/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2.sources && \
    DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-jazzy-ament-index-cpp \
      ros-jazzy-controller-interface \
      ros-jazzy-controller-manager \
      ros-jazzy-control-msgs \
      ros-jazzy-geometry-msgs \
      ros-jazzy-hardware-interface \
      ros-jazzy-imu-sensor-broadcaster \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-joy \
      ros-jazzy-launch \
      ros-jazzy-launch-ros \
      ros-jazzy-mujoco-vendor \
      ros-jazzy-pluginlib \
      ros-jazzy-rclcpp \
      ros-jazzy-rclcpp-lifecycle \
      ros-jazzy-realtime-tools \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rosidl-default-runtime \
      ros-jazzy-rviz2 \
      ros-jazzy-sensor-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-std-srvs \
      ros-jazzy-xacro \
      ros-jazzy-generate-parameter-library \
      libeigen3-dev \
      libglfw3-dev \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.aliyun.com/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2.sources && \
    apt update && apt install -y ros-jazzy-rmw-zenoh-cpp && rm -rf /var/lib/apt/lists/*

# 源码编译改为容器内增量（挂载 ai_sapiens 到 /root/ros2_ws/src/ai_sapiens，
# colcon build 在容器内跑，避免源码改动触发镜像全量重建），见 docker/container.sh
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1'" >> ~/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
    echo "alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'">> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
