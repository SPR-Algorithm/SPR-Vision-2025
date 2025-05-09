# SPR2025 Vision Project

中国石油大学SPR战队2025赛季视觉项目主仓库

基于中南大学2024年视觉开源，很遗憾我们的开发规模不足以支持对现有的系统再做出除内部适配外更多的改动

该项目在原rm_vision项目上扩展了自瞄选板、能量机关识别与预测、哨兵定位、自主导航等功能，为RoboMaster机器人实现了一套通用的算法框架

# 各兵种针对部署备忘录
各台车的相机内参以及相机-云台变换尺寸均需要针对实际进行修改

# 全流程部署指南

## 1. 安装Ubuntu 22.04 LTS
强烈建议安装时选择Minimal Installation，可以少点没用的东西。

## 2. fishros安装ros2 humble desktop
```
wget http://fishros.com/install -O fishros && . fishros
sudo apt update && sudo apt upgrade
sudo apt-get install ros-humble-image-transport-plugins
sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-serial-driver
```

## 3.使用一键安装脚本安装依赖
```
chmod +x install_from_zips.sh
./install_from_zips.sh
```

## 4. 编译安装CH341驱动并配置串口
```
sudo apt remove brltty
```
![image](https://github.com/user-attachments/assets/c4abf805-2ec8-453b-90ed-23c1549c6840)
下载并按照压缩包内readme配置串口驱动

如果提示没有gcc-12，使用apt安装gcc-12
如果提示insmod: ERROR: could not insert module ch341.ko: Unknown symbol in module

则进行
```
modinfo ch341.ko |grep depends
depends:        usbserial
```
然后
```
sudo modprobe usbserial
```
问题应该解决

### 安装完成后：

验证：lsmod | grep ch34
ch341                  24576  0
usbserial              69632  1 ch341

## 以下为手动安装方法，一键安装脚本遇到问题时，可单独对照进行debug

### 安装spdlog库（版本1.14）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
cmake之后如下方fmt一样，在CmakeCache.txt里面添加-fPIC选项

### 安装FMT库（版本10.2.1）
//已完成
修改armor\_detector节点里armor\_detector.cpp的代码，在include里添加#include \<fmt/format.h>

压缩包解压后cd进去

修改CMakeLists.txt，在指定位置添加如下行：
```
#Add -fPIC option
add_compile_options(-fPIC)
```

![](docs/p3XZKold31xFgpCrUgQssCxuooFgjEb0PcBcbobKgNI=.png)

cd进去执行：
```
mkdir build && cd build
cmake ..
make
sudo make install
```
然后编辑build/CMakeCache.txt，在此处添加如下参数

![](docs/BSRyhPmF56mW5Yb9JATeD7Ye1237wWlQ6FqYtxfkzeo=.png)

然后重新在build目录执行：
```
make
sudo make install
```
如此，编译应该通过

### 安装g2o库（版本20230806_git）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
性能过差卡死解决方案：将make -j改为make -j4或更小的数字，任何时候遇到编译性能问题都可如此尝试

编译时同样需要添加-fPIC选项！

### 安装Ceres-Solver库（版本2.0.0）（可能2.2.0）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
rosdep提示缺少ceres是正常现象不必理会，确保apt中libceres的版本为2.0.0
若编译中Cmake提示找不到tbb相关文件，则卸载当前的libtbb，并按顺序安装libtbb2，libtbb2-dev，libtbbmalloc2-dev

### 添加串口&相机的权限规则
```
sudo cp camera.rules  /etc/udev/rules.d/
sudo cp serial.rules  /etc/udev/rules.d/
```
添加后重启生效

## 5.安装OpenVINO
引擎搜索Install OpenVINO，选择介于2022-2024之间的版本，Distribution选择APT方式，并按照官网指示完成安装。

## 6.部署测试SPR-Vision-2025
在src上层运行
```
rosdepc update
rosdepc install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 4
```
成功编译后，按照如下方式启动：
```
source install/setup.bash
ros2 launch rm_bringup bringup.launch_mvtest.py
```

## 7. 启动相机节点与调试环境设置与相机标定
标定板PDF生成网站：
https://calib.io/pages/camera-calibration-pattern-generator
注意打印时一定要避免因打印页面缩放导致的尺寸误差！

使用USB连接相机

在工作文件夹运行

source install/setup.bash

👆记得这个命令每次新建终端都要执行一次

ros2 run mindvision\_camera mindvision\_camera\_node

rqt添加

Plugins->Visualization->Image View

Plugins->Configurations->Dynamic Reconfigure

若没找到话题和节点记得点刷新

### 标定部分

安装
```
sudo apt install ros-humble-camera-calibration
```
然后运行
```
ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.03 image:=/image_raw camera:=/mv_camera
```
注意，标定板的规格与尺寸大小需要根据实际情况做出相应修改!
按照进度条指示完全移动标定板，尽量使进度条变满，差不多后点击Calibrate；
计算完成后点击Save，结果文件位于/tmp/calibrationdata.tar.gz

## 8. 单独启动识别节点调试

ros2 run armor\_detector armor\_detector\_node

rqt选择/armor\_detector节点配置，打开debug选项，可在左侧image view看到/detector/result\_img

调整相机对焦和光圈，使其能识别出装甲板且置信度稳定在100%

## 9.串口协议通信调试
所有的数据包均统一为16位的FixPacket，其中帧头0xFF，帧尾0xFE；
发送给电控格式为：帧头0xFF，开火（1字节），Yaw（4字节），Pitch（4字节），Distance（4字节），留空（1字节），帧尾0xFE
从电控接收格式为：帧头0xFF，颜色（1字节），填充（2字节）Pitch（4字节），Yaw（4字节），帧尾0xFE，留空（3字节）
遇到通信错误导致

## 10.云台-相机描述模型尺寸修改
右手系，相机镜片平面中心与云台转动轴中心的相对位置，根据兵种情况修改xyz

## 11.代码编译部署错误备忘
若遇到类似Something went wrong while looing up transform之类的串口通信问题，按照
检查硬件连接->CuteCom检查串口接收通信工作情况->检查数据校验是否成功
的步骤，依次检查与下位机的通信情况

CMake Error at /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:240 (list):
list index: 1 out of range (-1, 0)
的问题，则表示路径中有非Unicode字符，将工作目录移动至无中文路径中删除build重新编译。

若遇到类似error while loading shared libraries: libg2o_core.so: cannot open shared object file
等g2o库等无法找到的问题，首先确保g2o已按照-fPIC参数正确编译并安装成功，若问题仍然存在，则使用如下命令编辑该文件：
sudo gedit /etc/ld.so.conf
并在文本编辑器中添加如下行：
/usr/local/lib
保存并退出，运行
sudo ldconfig
如此，问题应该解决。
原文链接：https://blog.csdn.net/weixin_38258767/article/details/106875766
<details>
<summary>以下是原项目仓库中的部署指南：</summary>
## 一、项目结构

*表示不在本仓库中直接提供，这部分模块已在[CSU-RM-Sentry](https://github.com/baiyeweiguang/CSU-RM-Sentry)开源

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   ├── livox_ros_driver2 (*Livox激光雷达驱动)
│   │
│   ├── rm_camera_driver (相机驱动)
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│
├── rm_rune (打符算法)
│
├── rm_localization (*定位算法)
│
├── rm_perception (*感知算法)
│
├── rm_navigation (*导航算法)
│
├── rm_decision (*自主决策算法)
│
├── rm_utils (工具包) 
│   ├── math (包括PnP解算、弹道补偿等)
│   │
│   └── logger (日志库)
│
└── rm_upstart (自启动配置)
```

## 二、环境

如果你不需要完整功能，可以直接把相关的功能包删除掉

### 1. 基础
- Ubuntu 22.04
- ROS2 Humble
- Mindvision相机驱动

### 2. 自瞄 
- fmt库
  ```bash
  sudo apt install libfmt-dev
  ```
- Sophus库1.22.10 (G2O库依赖)
   ```bash
   git clone https://github.com/strasdat/Sophus
   cd Sophus
   mkdir build && cd build
   cmake ..
   make -j
   sudo make install
   ```
- G2O库 (优化装甲板Yaw角度)
    ```bash
    sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```
### 3. 能量机关
- OpenVINO库 (能量机关识别)
  
   参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

- Ceres库 (能量机关曲线拟合)
    ```bash
    sudo apt install libceres-dev
    ```

### 4. 导航
- Livox SDK2

  参考[Livox官方仓库](https://github.com/Livox-SDK/Livox-SDK2)

- Navigation2 (导航)
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```

- PCL库 (导航时对点云的处理)
    ```
    sudo apt install libpcl-dev
    ```

### 5. 其他

本文档中可能有缺漏，如有，可以用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y

```

## 三、编译与运行

修改rm_bringup/config/launch_params.yaml，选择需要启动的功能

```bash
# 编译
colcon build --symlink-install --parallel-workers 4 #本仓库包含的功能包过多，建议限制同时编译的线程数
# 运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

默认日志和内录视频路径为`~/fyt2024-log/`

> 我们的日志库是用fmt搓的，不使用ros2的日志库


## 四、自启动

- 编译程序后，进入rm_upstart文件夹

```bash
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出
```
查看screen.output或~/fyt2024-log下的日志
```  

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```

## 维护者及开源许可证
Maintainer : SPR Algorithm

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

感谢每个赛季视觉组的每一个成员的付出，感谢以下开源项目：
- [rm_vision](https://gitlab.com/rm_vision) rv是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [rmoss](https://github.com/robomaster-oss/rmoss_core) rmoss项目为RoboMaster提供通用基础功能模块包，本项目的串口驱动模块基于rmoss_base进行开发
- [沈阳航空航天大学TUP战队2022赛季步兵视觉开源](https://github.com/tup-robomaster/TUP-InfantryVision-2022) 为本项目的能量机关识别与预测算法提供了参考
- [沈阳航空航天大学YOLOX关键点检测模型](https://github.com/tup-robomaster/TUP-NN-Train-2) 提供了本项目能量机关识别模型训练代码
- [四川大学OpenVINO异步推理代码](https://github.com/Ericsii/rm_vision-OpenVINO) 提供了本项目能量机关识别模型部署的代码
- [上海交通大学自适应扩展卡尔曼滤波](https://github.com/julyfun/rm.cv.fans/tree/main) 使用Ceres自动微分功能，自动计算Jacobian矩阵

## 更新日志

- 参考上交开源，实现了EKF的自动求Jacobian矩阵
- 增加了粒子滤波器，为状态估计提供新的选择
- 修复了打符崩溃的问题（OpenVINO在推理时不能创建新的InferRequest，通过互斥锁解决）
- 将自瞄解算修改为定时器回调，固定解算的频率
- 增加手动补偿器ManualCompensator
- 重写PnP选解逻辑
- 修改了BA优化的代码，抽象出新的类ArmorPoseEstimator
- 为中国石油大学SPR机器人队代码做了适应性部署
</details>
