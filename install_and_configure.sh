#!/bin/bash
echo "[务必确保目录命名为SPR-Vision-2026]"

echo "[请求sudo权限]"
sudo -v

cd utils

if [ $? -ne 0 ]; then
    echo "[请求sudo权限失败]"
    exit 1
fi

echo "[启动apt更新]"
sudo apt update && sudo apt upgrade -y
sudo apt-get install -y gcc-12
sudo apt-get install -y unzip
sudo apt-get install -y ros-humble-image-transport-plugins
sudo apt-get install -y ros-humble-asio-cmake-module
sudo apt-get install -y ros-humble-foxglove-bridge
sudo apt-get install -y ros-humble-serial-driver
sudo apt-get install -y ros-humble-xacro
sudo apt-get install -y ros-humble-camera-calibration
sudo apt-get install -y libgoogle-glog-dev
sudo apt-get install -y libmetis-dev
sudo apt-get install -y libsuitesparse-dev
sudo apt-get remove -y brltty
sudo apt install -y ros-humble-camera-info-manager
sudo apt install -y ros-humble-camera-info-manager-dbgsym
sudo apt install -y ros-humble-camera-calibration
echo "[apt更新完成]"

echo "[开始调整swap分区大小]"
sudo swapoff /swapfile
sudo rm /swapfile
sudo fallocate -l 40G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo "[swap分区大小调整完成]"

echo "[开始安装依赖]"
cd libs
# 遍历当前目录下的所有zip文件
for ZIP_FILE in *.zip; do
    # 检查是否存在zip文件
    if [[ -f "$ZIP_FILE" ]]; then
        # 解压zip文件到同名目录
        unzip -o "$ZIP_FILE"

        # 进入解压后的目录
        cd "${ZIP_FILE%.zip}" || continue
        # 替换cere的cmake里面的FindTBB
        if [[ "${ZIP_FILE%.zip}" == "ceres-solver-2.0.0" ]]; then
            echo "Fixing FindTBB for ceres-solver"
            cp ../FindTBB_new.cmake ./cmake/FindTBB.cmake
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
cd ..
echo "[依赖安装完成]"

echo "[开始安装CH341驱动]"
unzip -o CH341SER_LINUX.ZIP
cd CH341SER_LINUX/driver
make
sudo make install
cd ..
cd ..
echo "[CH341驱动安装完成]"

echo "[开始添加udev规则]"
sudo cp ./rules/camera_mv.rules /etc/udev/rules.d/
sudo cp ./rules/camera_hik.rules /etc/udev/rules.d/
sudo cp ./rules/serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "[udev规则添加完成]"

echo "[开始安装OpenVINO]"
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
sudo apt update
sudo apt install -y openvino-2023.3.0
cd neo
# wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.17384.11/intel-igc-core_1.0.17384.11_amd64.deb
# wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.17384.11/intel-igc-opencl_1.0.17384.11_amd64.deb
# wget https://github.com/intel/compute-runtime/releases/download/24.31.30508.7/intel-level-zero-gpu-dbgsym_1.3.30508.7_amd64.ddeb
# wget https://github.com/intel/compute-runtime/releases/download/24.31.30508.7/intel-level-zero-gpu_1.3.30508.7_amd64.deb
# wget https://github.com/intel/compute-runtime/releases/download/24.31.30508.7/intel-opencl-icd-dbgsym_24.31.30508.7_amd64.ddeb
# wget https://github.com/intel/compute-runtime/releases/download/24.31.30508.7/intel-opencl-icd_24.31.30508.7_amd64.deb
# wget https://github.com/intel/compute-runtime/releases/download/24.31.30508.7/libigdgmm12_22.4.1_amd64.deb
sudo dpkg -i *.deb
cd ..
echo "[OpenVINO安装完成]"

echo "[开始配置自启动脚本]"
chmod +x ./start/*.sh
echo "[自启动脚本配置完成]"

echo "[开始测试编译工作空间]"
cd .. && cd Main_ws
colcon build --symlink-install --parallel-workers 4
echo "[测试编译工作空间完成]"

echo "[脚本运行完毕]"
