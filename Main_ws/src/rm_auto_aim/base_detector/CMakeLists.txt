cmake_minimum_required(VERSION 3.8)
project(base_detector)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(base_detector_sub src/base_detector_sub.cpp)

# 链接依赖库
ament_target_dependencies(base_detector_sub
  rclcpp
  sensor_msgs
  cv_bridge
)

# 链接 OpenCV 库
target_link_libraries(base_detector_sub
  ${OpenCV_LIBS}
)

# 安装目标
install(TARGETS
  base_detector_sub
  DESTINATION lib/${PROJECT_NAME}
)

# 安装源文件（可选）
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
