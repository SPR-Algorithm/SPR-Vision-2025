// Copyright 2025 ZhangSuHang

#include <openvino_armor_detector/openvino_detect_node.hpp>
#include <fmt/format.h>
#include <rmw/qos_profiles.h>
// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rmoss_util/url_resolver.hpp>

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim
{
OpenVINODetectNode::OpenVINODetectNode(rclcpp::NodeOptions options)
: Node("openvino_detect_node", options.use_intra_process_comms(true))
{

  FYT_REGISTER_LOGGER("armor_detector", "~/fyt2024-log", INFO);
  FYT_INFO("armor_detector", "Starting ArmorDetectorNode!");

  RCLCPP_INFO(this->get_logger(), "Initializing detect node");

  RCLCPP_INFO(this->get_logger(), "Initializing OpenVINO");
  detector_ = nullptr;
  //实例化检测器
  model_path_xml_= this->declare_parameter("model_path_xml", "package://armor_detector_network/model/0708.xml");
  model_path_bin_= this->declare_parameter("model_path_bin", "package://armor_detector_network/model/0708.bin");
  std::string device="GPU";
  detector_=std::make_shared<OpenVINODetector>(model_path_xml_,model_path_bin_,device);
  detector_->set_callback(
    std::bind(
      &OpenVINODetectNode::openvino_detect_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  // this->init_detector();
  if (!detector_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenVINO");
    return;
  }
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  detect_color_ = this->declare_parameter("detect_color", 1, param_desc);
  // qos?
  auto use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);


  transport_type_ =
    this->declare_parameter("detector.subscribe_compressed", false) ? "compressed" : "raw";
  RCLCPP_INFO(
    this->get_logger(), "transport_type: %s", transport_type_.c_str());

  // Debug mode handler
  RCLCPP_INFO(this->get_logger(), "Setup debug_mode handler");
  debug_mode_ = this->declare_parameter("debug_mode", true);
  if (debug_mode_) {
    this->create_debug_publishers();
  }
  // Regiter debug mode param handler
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback(
    "debug_mode", [this](const rclcpp::Parameter & p) {
      this->debug_mode_ = p.as_bool();
      this->debug_mode_ ? this->create_debug_publishers() : this->destroy_debug_publishers();
    });

  RCLCPP_INFO(this->get_logger(), "Setup ROS subs pubs");
  // Armors publisher
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
    "armor_detector/armors",
    rclcpp::SensorDataQoS());
  
  // Transform initialize
  odom_frame_ = this->declare_parameter("target_frame", "odom");
  imu_to_camera_ = Eigen::Matrix3d::Identity();

  // // Tricks to make pose more accurate
  use_ba_ = this->declare_parameter("use_ba", true);

  // Visualization Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.y = 0.15;
  armor_marker_.scale.z = 0.12;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_detector/marker", 10);

  // Camera handler
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
     "/camera_info", use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(1),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      this->cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_);
      armor_pose_estimator_->enableBA(use_ba_);
      this->measure_tool_ =
      std::make_unique<MonoMeasureTool>(
        std::vector<double>(
          this->cam_info_->k.begin(),
          this->cam_info_->k.end()), this->cam_info_->d);

      RCLCPP_INFO(
        this->get_logger(),
        fmt::format(
          "Camera intrinsic: {} \ncamera distortion: {}", fmt::join(this->cam_info_->k, " "),
          fmt::join(this->cam_info_->d, " "))
        .c_str());

      // Release subscription
      this->cam_info_sub_.reset();
    });


  img_sub_ = std::make_shared<image_transport::Subscriber>(
    image_transport::create_subscription(
      this, "/image_raw",
      std::bind(&OpenVINODetectNode::img_callback, this, std::placeholders::_1),
      transport_type_,
      use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  RCLCPP_INFO(this->get_logger(), "Subscribing to %s", img_sub_->getTopic().c_str());

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  RCLCPP_INFO(this->get_logger(), "Initializing finished.");
}


void OpenVINODetectNode::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)//图像回调函数,订阅到图像消息后开始推理
{
  detect_color_= get_parameter("detect_color").as_int();
  //Get the transform from odom_aim to gimbal
  try {
    // ? 可能是tf的数据传输，或延迟导致detector节点无法查询到变换
    rclcpp::Time target_time = img_msg->header.stamp;
    //std::cout<<odom_frame_<<std::endl;
    auto odom_to_gimbal = tf2_buffer_->lookupTransform(
        odom_frame_, img_msg->header.frame_id, target_time,
        rclcpp::Duration::from_seconds(0.01));
    auto msg_q = odom_to_gimbal.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
    imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
        tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
        tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
        tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
        tf2_matrix.getRow(2)[2];
  } catch (...) {
    std::cout<<"armor_detector Something Wrong when lookUpTransform"<<std::endl;
    return;
  }

  // limits request size
  while (detect_requests_.size() > 5) {
    detect_requests_.front().get();
    detect_requests_.pop();
  }

  auto timestamp = rclcpp::Time(img_msg->header.stamp);
  frame_id_ = img_msg->header.frame_id;
  auto img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
  //RCLCPP_INFO(this->get_logger(),"Image_size: %d x %d",img.cols,img.rows);
  
  cv::Mat resize_img = detector_->letterbox(img); 

  // push image to detector
  detect_requests_.push(std::move(detector_->push_input(resize_img,detect_color_, timestamp.nanoseconds())));


}

void OpenVINODetectNode::openvino_detect_callback(
  const std::vector<ArmorObject> & objs, int64_t timestamp_nanosec,
  const cv::Mat & resized_img){
    
  detect_color_= get_parameter("detect_color").as_int();
  //std::cout<<"进入推理回调"<<std::endl;
  if (measure_tool_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "No camera_info recieve yet.");
    return;
  }

  // Used to draw debug info
  cv::Mat debug_img;
  if (debug_mode_) {
    resized_img.copyTo(debug_img);
  }

  auto timestamp = rclcpp::Time(timestamp_nanosec);
  armors_msg.header.frame_id = armor_marker_.header.frame_id = text_marker_.header.frame_id = frame_id_;
  armors_msg.header.stamp    = armor_marker_.header.stamp    = text_marker_.header.stamp    = timestamp;


  //清空装甲板容器消息
  armors_msg.armors.clear();
  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;

  // //遍历objects，排除不符合颜色的装甲板
  // for (auto & obj : objs){
  //   //std::cout<<"检测到目标"<<std::endl;
  //   if (detect_color_ == 0 && obj.color !=0) {
  //     continue;
  //   } else if (detect_color_ == 1 && obj.color != 1) {
  //     continue;
  //   }

  //   rm_interfaces::msg::Armor armor;//单个装甲板消息

  //   cv::Point3f target_position;
  //   cv::Mat target_rvec;

  //   if (!measure_tool_->calc_armor_target(obj, target_position, target_rvec)) {//判断大小装甲板，对装甲板进行位姿结算
  //     RCLCPP_WARN(this->get_logger(), "Calc target failed.");
  //   }
    
  //   cv::Mat rot_mat;
  //   cv::Rodrigues(target_rvec, rot_mat);
  //   tf2::Matrix3x3 tf_rot_mat(
  //   rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
  //   rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
  //   rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
  //   tf2::Quaternion tf_quaternion;
  //   tf_rot_mat.getRotation(tf_quaternion);
    
  //   //Fill the armor
  //   armor.type="invalid";
  //   armor.number =  kArmorNames[static_cast<int>(obj.label)];
  //   armor.pose.position.x = target_position.x;
  //   armor.pose.position.y = target_position.y;
  //   armor.pose.position.z = target_position.z;
  //   armor.pose.orientation.x = tf_quaternion.x();
  //   armor.pose.orientation.y = tf_quaternion.y();
  //   armor.pose.orientation.z = tf_quaternion.z();
  //   armor.pose.orientation.w = tf_quaternion.w();
  //   armor.distance_to_image_center = measure_tool_->calc_distance_to_center(obj);  // the distance to image center


  //   if(obj.type == ArmorType::LARGE){//新赛季只有英雄和基地是大装甲板
  //     armor.type="large";
  //     armor_marker_.scale.y=0.23;
  //   }
  //   else{
  //     armor.type="small";
  //     armor_marker_.scale.y=0.135;
  //   }
  //   //单个armor填入消息容器
  //   armors_msg.armors.push_back(std::move(armor));

  //   // Fill the markers
  //   armor_marker_.id++;
  //   armor_marker_.pose = armor.pose;
  //   text_marker_.id++;
  //   text_marker_.pose.position = armor.pose.position;
  //   text_marker_.pose.position.y -= 0.1;
  //   text_marker_.text = std::string(armor.number);
  //   marker_array_.markers.emplace_back(armor_marker_);
  //   marker_array_.markers.emplace_back(text_marker_);

  //   if (debug_mode_) {
  //     if (debug_img.empty()) {
  //       // Avoid debug_mode change in processing
  //       continue;
  //     }

  //     // Draw armor
  //     for (size_t i = 0; i < 8; i += 2) {
  //       cv::line(
  //                                       //前一个点                                              //后一个点
  //         debug_img, cv::Point2f(obj.landmarks[i], obj.landmarks[i + 1]), cv::Point2f(obj.landmarks[(i + 2) % 8],obj.landmarks[(i + 3) % 8]),
  //         cv::Scalar(255, 48, 48), 1);
  //     }

  //   std::string armor_color;
  //   switch (obj.color) {
  //     case 0:
  //       armor_color = "Red";
  //       break;
  //     case 1:
  //       armor_color = "Blue";
  //       break;
  //     case 2:
  //       armor_color = "Grey";
  //       break;
  //     case 3:
  //       armor_color = "Purple";
  //       break;
  //     default:
  //       armor_color = "UNKOWN";
  //       break;
  //   }

  //     std::string armor_key = fmt::format("{} {}", armor_color, obj.label);
  //     cv::putText(
  //       debug_img, armor_key, cv::Point2i(obj.landmarks[0], obj.landmarks[1]), cv::FONT_HERSHEY_SIMPLEX, 0.8,
  //       cv::Scalar(0, 255, 255),
  //       2);
  //   }

  // }

  for (auto & obj : objs){
    //std::cout<<"检测到目标"<<std::endl;
    if (detect_color_ == 0 && obj.color !=0) {
      continue;
    } else if (detect_color_ == 1 && obj.color != 1) {
      continue;
    }

    rm_interfaces::msg::Armor armor_msg;//单个装甲板消息
    armor_msg = armor_pose_estimator_->extractArmorPoses(obj, imu_to_camera_);
    armors_msg.armors.push_back(std::move(armor_msg));


    if (debug_mode_) {
      if (debug_img.empty()) {
        // Avoid debug_mode change in processing
        continue;
      }

      // Draw armor
      for (size_t i = 0; i < 8; i += 2) {
        cv::line(
                                        //前一个点                                              //后一个点
          debug_img, cv::Point2f(obj.landmarks[i], obj.landmarks[i + 1]), cv::Point2f(obj.landmarks[(i + 2) % 8],obj.landmarks[(i + 3) % 8]),
          cv::Scalar(255, 48, 48), 1);
      }

    std::string armor_color;
    switch (obj.color) {
      case 0:
        armor_color = "Red";
        break;
      case 1:
        armor_color = "Blue";
        break;
      case 2:
        armor_color = "Grey";
        break;
      case 3:
        armor_color = "Purple";
        break;
      default:
        armor_color = "UNKOWN";
        break;
    }

      std::string armor_key = fmt::format("{} {}", armor_color, obj.label);
      cv::putText(
        debug_img, armor_key, cv::Point2i(obj.landmarks[0], obj.landmarks[1]), cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(0, 255, 255),
        2);
    }

  }

  //发布装甲板消息
  armors_pub_->publish(std::move(armors_msg));

  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;
  armor_marker_.header = text_marker_.header = armors_msg.header;
  // Fill the markers
  for (const auto &armor : armors_msg.armors) {
    armor_marker_.pose = armor.pose;
    armor_marker_.id++;
    text_marker_.pose.position = armor.pose.position;
    text_marker_.id++;
    text_marker_.pose.position.y -= 0.1;
    text_marker_.text = armor.number;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_array_.markers.emplace_back(text_marker_);
  }
  //发布marker消息
  publishMarkers();

  //调试模式下发布调试图像
  if (debug_mode_) {
  if (debug_img.empty()) {
    // Avoid debug_mode change in processing
    return;
  }

  //draw the results on debugimage
  cv::circle(
    debug_img, cv::Point2i(
      cam_info_->width  / (2.0*(1280.0 / 640.0)),
      cam_info_->height / (2.0*(1024.0 / 640.0))), 4, cv::Scalar(255, 0, 0), -1);

  auto end = this->get_clock()->now();
  auto duration = end.seconds() - timestamp.seconds();
  std::string latency = fmt::format("Latency: {:.3f}ms", duration * 1000);
  cv::putText(
    debug_img, latency, cv::Point2i(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
    cv::Scalar(0, 255, 255),
    2);

  debug_img_pub_.publish(cv_bridge::CvImage(armors_msg.header, "rgb8", debug_img).toImageMsg());
  }

}

//visualization
void OpenVINODetectNode::publishMarkers() noexcept {
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action =
      armors_msg.armors.empty() ? Marker::DELETEALL : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

void OpenVINODetectNode::create_debug_publishers()
{
  debug_img_pub_ = image_transport::create_publisher(this, "detector/debug_img");
}

void OpenVINODetectNode::destroy_debug_publishers()
{
  debug_img_pub_.shutdown();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::OpenVINODetectNode)