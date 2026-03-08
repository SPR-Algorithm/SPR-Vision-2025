# SPR Vision 2026 - RoboMaster 视觉自瞄系统 Docker 镜像
# 基于 Ubuntu 22.04 LTS 和 ROS2 Humble (x86_64 架构)

FROM --platform=linux/amd64 ubuntu:22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV ROS_DISTRO=humble
ENV WORKSPACE_PATH=/workspace/SPR-Vision-2026

# 创建工作目录
WORKDIR $WORKSPACE_PATH

# 设置时区
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 更新系统并安装基础依赖
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    unzip \
    gcc-12 \
    libgoogle-glog-dev \
    libmetis-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS2 Humble Desktop
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-image-transport-plugins \
    ros-humble-asio-cmake-module \
    ros-humble-foxglove-bridge \
    ros-humble-serial-driver \
    ros-humble-xacro \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && rosdep update

# 安装 OpenVINO
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2024.list && \
    apt-get update && \
    apt-get install -y openvino-2024.6.0 && \
    rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    rm -rf /var/lib/apt/lists/*

# 复制项目文件
COPY . $WORKSPACE_PATH/

# 设置工作目录权限
RUN chmod -R 755 $WORKSPACE_PATH

# 安装第三方库依赖 (x86_64 架构)
WORKDIR $WORKSPACE_PATH/utils/libs
RUN for ZIP_FILE in *.zip; do \
        if [ -f "$ZIP_FILE" ]; then \
            unzip -o "$ZIP_FILE" && \
            cd "${ZIP_FILE%.zip}" && \
            if [ "${ZIP_FILE%.zip}" = "ceres-solver-2.0.0" ]; then \
                cp ../FindTBB_new.cmake ./cmake/FindTBB.cmake; \
            fi && \
            mkdir -p build && \
            cd build && \
            cmake -DCMAKE_CXX_FLAGS="-fPIC -m64" -DCMAKE_SYSTEM_PROCESSOR=x86_64 .. && \
            make -j2 && \
            make install && \
            cd ../..; \
        fi; \
    done

# 安装 Intel OpenCL 运行时 (NEO)
WORKDIR $WORKSPACE_PATH/utils/neo
RUN dpkg -i *.deb || apt-get install -f -y

# 复制 udev 规则文件 (在容器中可能不需要，但保留以备不时之需)
WORKDIR $WORKSPACE_PATH/utils
RUN mkdir -p /etc/udev/rules.d/ && \
    cp rules/*.rules /etc/udev/rules.d/ || echo "udev rules copied or not available"

# 设置启动脚本权限
RUN chmod +x start/*.sh

# 编译 ROS2 工作空间 (针对 x86 架构优化)
WORKDIR $WORKSPACE_PATH/Main_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=\"-O2 -march=native -mtune=native\""

# 设置环境变量和启动脚本
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $WORKSPACE_PATH/Main_ws/install/setup.bash" >> /root/.bashrc && \
    echo "cd $WORKSPACE_PATH" >> /root/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source $WORKSPACE_PATH/Main_ws/install/setup.bash\n\
cd $WORKSPACE_PATH/Main_ws\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 暴露常用端口 (根据需要调整)
EXPOSE 8080 9090

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]

# 默认命令
CMD ["bash"]

# 元数据标签
LABEL maintainer="SPR算法组"
LABEL description="SPR Vision 2026 - RoboMaster 视觉自瞄系统"
LABEL version="2026"
LABEL ros_distro="humble"