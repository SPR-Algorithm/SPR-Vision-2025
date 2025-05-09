#!/bin/bash

# 提示用户输入sudo密码
echo "This script requires sudo privileges. Please enter your password:"
sudo -v

cd Utils

# 检查sudo是否成功
if [ $? -ne 0 ]; then
    echo "Failed to obtain sudo privileges. Exiting."
    exit 1
fi

# 更新和升级系统包
sudo apt update && sudo apt upgrade -y

# 补全安装依赖库并解决冲突·
sudo apt-get install -y unzip
sudo apt-get install -y ros-humble-image-transport-plugins
sudo apt-get install -y ros-humble-asio-cmake-module
sudo apt-get install -y ros-humble-foxglove-bridge
sudo apt-get install -y ros-humble-serial-driver
sudo apt-get install -y ros-humble-camera-calibration
sudo apt-get install -y libgoogle-glog-dev
sudo apt-get install -y libmetis-dev
sudo apt-get install -y libsuitesparse-dev
sudo apt-get remove -y brltty

# 遍历当前目录下的所有zip文件
for ZIP_FILE in *.zip; do
    # 检查是否存在zip文件
    if [[ -f "$ZIP_FILE" ]]; then
        # 解压zip文件到同名目录
        unzip "$ZIP_FILE"

        # 进入解压后的目录
        cd "${ZIP_FILE%.zip}" || continue
        # 替换cere的cmake里面的FindTBB
        if [[ "${ZIP_FILE%.zip}" == "ceres-solver-2.0.0" ]]; then
            echo "change FindTBB"
            cp ../FindTBB_new.cmake ./FindTBB.cmake
        fi

        # 创建build目录并进入
        mkdir build
        cd build || continue

        # 执行cmake和make命令
        cmake -DCMAKE_CXX_FLAGS="-fPIC" ..
        make -j4
        sudo make install

        # 返回上级目录以处理下一个zip文件
        cd ../..
    else
        echo "No zip files found in the current directory."
    fi
done

# 复制规则文件到/etc/udev/rules.d/
sudo cp ./rules/camera.rules /etc/udev/rules.d/
sudo cp ./rules/serial.rules /etc/udev/rules.d/

# 重新加载udev规则
sudo udevadm control --reload-rules && sudo udevadm trigger