# 反投影（Reprojection）功能指南

本文档说明在 `armor_solver_classic` 中加入反投影可视化相关的改动、使用方法和注意事项。目标是将检测到的装甲板二维像素点反投影到三维（基于 PnP / 预测位姿），并在相机图像上可视化反投影结果以便调试。

---

## 1. 主要修改清单

- Solver 类（`armor_solver.hpp` / `armor_solver.cpp`）
  - 新增 `setCameraParameters`：从 `sensor_msgs::msg::CameraInfo` 提取相机内参与畸变系数并保存到 Solver 内部。
  - 实现 `reproject_single_armor`：对单个装甲板（4/3 点）进行 3D->2D 反投影计算。
  - 实现 `reproject_all_armors`：对检测到的所有装甲板调用反投影并返回像素点集合。
  - 实现 `calculate_reprojection_error`：计算原始检测角点与反投影点之间的像素误差（用于评估精度）。
  - 移除硬编码相机参数，改为通过 `camera_info` 动态加载。

- Node 节点（`armor_solver_node.hpp` / `armor_solver_node.cpp`）
  - 订阅 `camera_info` 话题以获取相机参数（完成后调用 Solver::setCameraParameters）。
  - 订阅 `image_raw`（或 image_transport）的图像话题以获取图像并在上面绘制可视化结果。
  - 新增可视化方法：
    - `publishReprojectionImage`：将绘制好的可视化图像发布为 `/reprojection/image`（或带命名空间的对应话题）。
    - `drawReprojectedArmors`：在图像上绘制反投影的 3D 点集轮廓与角点。
  - 在 `armorsCallback`（接收检测结果的回调）内调用反投影和绘制函数，最后发布可视化图像。

- 依赖项（`package.xml` / `CMakeLists.txt`）
  - 添加依赖：`cv_bridge`、`image_transport`、`sensor_msgs`、`OpenCV`（若尚未添加）。

- 配置文件（`armor_solver_classic_params.yaml`）
  - 新增或确保存在：

    ```yaml
    enable_reprojection_visualization: true
    reprojection_image_topic: /reprojection/image
    reprojection_line_color: [0, 255, 0]
    reprojection_point_color: [0, 255, 0]
    reprojection_line_thickness: 2
    reprojection_point_radius: 3
    ```

---

## 2. 功能特性

- 自动相机参数加载：从 `camera_info` 话题读取相机内参与畸变参数。
- 实时反投影可视化：在原始相机图像上绘制反投影后的装甲板轮廓与角点（默认绿色）。
- 多装甲板支持：支持 4 点装甲（数字装甲）和 3 点装甲（哨站/特殊形状）。
- 坐标系转换：使用 TF 做坐标系变换（camera_link <-> gimbal_link <-> odom 等）。
- 错误处理与边界检查：对投影结果做有效性检查（是否在图像内、除零保护等）。

---

## 3. 使用方法（快速上手）

1. 启动相机驱动并确保:
   - 相机发布 `sensor_msgs/msg/CameraInfo` （通常为 `/camera/camera_info` 或带命名空间的话题）。
   - 相机或驱动发布原始图像（如 `/camera/image_raw`）。

1. 启动检测/解算节点（通常通过 launch 文件）：

```bash
# 假设有一个 launch 文件包含 armor_solver 节点
ros2 launch rm_auto_aim armor_solver_launch.py
```

1. 查看反投影图像（两种方式）：

- 使用 `rqt_image_view` 或 `image_view` 订阅 `reprojection` 话题：

```bash
ros2 run rqt_image_view rqt_image_view
# 在 GUI 中选择 /<namespace>/reprojection/image
```

- 或者用 `rviz2` 同步显示图像（如果需要）。

1. 调试：观察控制台输出的 reprojection error，若误差较大，检查相机内参、TF 发布及 PnP 配置。

---

## 4. 可视化说明（图例）

- 绿色轮廓：反投影并按顺序连线后形成的装甲板边界。
- 绿色角点：装甲板的四个/三个角点（或用不同颜色区分原始检测点与反投影点）。
- 建议在调试模式下同时显示：原始检测轮廓（虚线/红色）与反投影轮廓（实线/绿色），便于对比。

示例绘制伪代码：

```cpp
// draw edges
cv::polylines(img, reprojected_contour, true, cv::Scalar(0,255,0), 2);
// draw corners
for (auto &p : reprojected_corners) cv::circle(img, p, 3, cv::Scalar(0,255,0), -1);
// optional: draw original detection in red
```

---

## 5. 注意事项与常见问题

- 相机信息话题：确保相机节点正确发布 `camera_info`（消息类型为 `sensor_msgs/msg/CameraInfo`）。Solver 在首次收到 `camera_info` 后会设置内参。
- 图像话题：确认图像发布在 `image_raw` 或节点配置的图像话题上。
- TF 变换：确保 TF 树包含必要的变换（例如 `camera_link` -> `gimbal_link`，以及 `gimbal_link` -> `odom` 或机器人基座），否则反投影的世界坐标转换会失败。
- 命名空间：若使用节点命名空间，话题名称会自动带上前缀，检查 launch 文件或 ros2 topic 列表确保名称一致。
- PnP/位姿来源：反投影结果依赖于用于反投影的 3D 位姿/估计，若位姿不准确（滤波器/预测错误），反投影会偏移。
- 性能：如果发布频率较高，绘制与转码会增加 CPU 开销；可通过配置开关 `enable_reprojection_visualization` 控制是否启用绘制。

---

## 6. 示例配置片段（在 `armor_solver_classic_params.yaml` 中）

```yaml
armor_solver:
  enable_reprojection_visualization: true
  reprojection_image_topic: /reprojection/image
  reprojection:
    line_color: [0, 255, 0]
    point_color: [0, 255, 0]
    line_thickness: 2
    point_radius: 3
```

---

如果你希望我把节点中具体的绘制代码或 camera_info 的订阅示例也补充到该文档中（带代码片段），或者直接在节点代码中生成一个小的 PR，我可以继续完成这些具体实现并运行一次快速检查。
