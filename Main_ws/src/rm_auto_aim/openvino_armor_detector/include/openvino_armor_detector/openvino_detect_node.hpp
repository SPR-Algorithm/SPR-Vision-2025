// Copyright 2025 ZhangSuHang

#ifndef OPENVINO_ARMOR_DETECTOR__OPENVINO_DETECT_NODE_HPP_
#define OPENVINO_ARMOR_DETECTOR__OPENVINO_DETECT_NODE_HPP_

#include <queue>
#include <future>
#include <vector>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <auto_aim_interfaces/msg/armors.hpp>
#include <openvino_armor_detector/openvino_detector.hpp>
#include <openvino_armor_detector/mono_measure_tool.hpp>

// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <openvino_armor_detector/armor_pose_estimator.hpp>

namespace rm_auto_aim
{

class OpenVINODetectNode : public rclcpp::Node
{
public:
  OpenVINODetectNode(
    rclcpp::NodeOptions options);

private:
  //void init_detector();//初始化检测器

  void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);//图像回调函数，触发推理请求

  void openvino_detect_callback(const std::vector<ArmorObject> & objs, int64_t timestamp_nanosec,const cv::Mat & resized_img);//推理回调函数，解码推理结果

  // Debug functions
  void create_debug_publishers();//创建调试发布者

  void destroy_debug_publishers();//销毁调试发布者

  void publishMarkers() noexcept;//marker发布者

private:

  std::string transport_type_;//传输类型
  std::string frame_id_;//帧ID
  // OpenVINO Detector
  int detect_color_;  // 0: red, 1: blue
  std::unique_ptr<OpenVINODetector> detector_;//OpenVINODetector指针
  std::queue<std::future<bool>> detect_requests_;//检测请求队列

  // Camera info
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;//相机信息
  std::unique_ptr<MonoMeasureTool> measure_tool_;//MonoMeasureTool指针

  // Pose Solver
  bool use_ba_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ROS
  rm_interfaces::msg::Armors armors_msg;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::shared_ptr<image_transport::Subscriber> img_sub_;

  // Debug publishers
  bool debug_mode_{false};
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher debug_img_pub_;

  // ReceiveData subscripiton
  std::string odom_frame_;
  Eigen::Matrix3d imu_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

};

}  // namespace rm_auto_aim

#endif  // OPENVINO_ARMOR_DETECTOR__OPENVINO_DETECT_NODE_HPP_
