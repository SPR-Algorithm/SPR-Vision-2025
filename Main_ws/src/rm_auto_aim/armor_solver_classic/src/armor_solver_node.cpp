// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_solver_node.hpp"

// std
#include <memory>
#include <vector>
#include <cmath>
// project
#include "armor_solver/motion_model.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"
#include <cv_bridge/cv_bridge.h>

namespace fyt::auto_aim {
//last cmd data
auto last_yaw=0.0;
auto last_pitch=0.0;
ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions &options)
: Node("armor_solver", options), solver_(nullptr) {
  // Register logger
  FYT_REGISTER_LOGGER("armor_solver", "~/fyt2024-log", INFO);
  FYT_INFO("armor_solver", "Starting ArmorSolverNode!");

  debug_mode_ = this->declare_parameter("debug", true);

  
  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc
  // measurement: p, y, d, yaw
  // f - Process function
  auto f = Predict(0.005);
  // h - Observation function
  auto h = Measure();
  // update_Q - process noise covariance matrix
  s2qx_max = declare_parameter("ekf.sigma2_q_x_max", 0.1);
  s2qy_max = declare_parameter("ekf.sigma2_q_y_max", 0.1);
  s2qz_max = declare_parameter("ekf.sigma2_q_z_max", 0.1);
  s2qyaw_max = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);

  s2qx_min = declare_parameter("ekf.sigma2_q_x_min", 0.05);
  s2qy_min = declare_parameter("ekf.sigma2_q_y_min", 0.05);
  s2qz_min = declare_parameter("ekf.sigma2_q_z_min", 0.05);
  s2qyaw_min = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);

  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  s2qd_zc_ = declare_parameter("ekf.sigma2_q_d_zc", 800.0);

  auto u_q = [this](const Eigen::VectorXd & x_p) {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_;
    // x = s2qx_, y = s2qy_, z = s2qz_, yaw = s2qyaw_, 
    double r = s2qr_; 
    double d_zc=s2qd_zc_;

    double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    double dx = pow(pow(vx,2)+pow(vy,2),0.5);
    double dy = abs(v_yaw);
    double x_factor = exp(-dy)*(s2qx_max-s2qx_min)+s2qx_min;
    double y_factor = exp(-dy)*(s2qy_max-s2qy_min)+s2qy_min;
    double z_factor = exp(-dy)*(s2qz_max-s2qz_min)+s2qz_min;
    double yaw_factor = exp(-dx)*(s2qyaw_max-s2qyaw_min)+s2qyaw_min;

    std::cout << "x_factor: " << std::fixed << std::setprecision(6) << x_factor << std::endl;

    double q_x_x = pow(t, 4) / 4 * x_factor, q_x_vx = pow(t, 3) / 2 * x_factor, q_vx_vx = pow(t, 2) * x_factor;
    double q_y_y = pow(t, 4) / 4 * y_factor, q_y_vy = pow(t, 3) / 2 * y_factor, q_vy_vy = pow(t, 2) * y_factor;
    double q_z_z = pow(t, 4) / 4 * z_factor, q_z_vz = pow(t, 3) / 2 * z_factor, q_vz_vz = pow(t, 2) * z_factor;
    double q_yaw_yaw = pow(t, 4) / 4 * yaw_factor, q_yaw_vyaw = pow(t, 3) / 2 * yaw_factor,
           q_vyaw_vyaw = pow(t, 2) * yaw_factor;
    double q_r = pow(t, 4) / 4 * r;
    double q_d_zc = pow(t, 4) / 4 * d_zc;
    // clang-format off
    //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw       r       d_za
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,          0,          0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0,          0,      0,
          0,      0,      q_y_y,  q_y_vy, 0,      0,      0,          0,          0,      0,
          0,      0,      q_y_vy, q_vy_vy,0,      0,      0,          0,          0,      0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,          0,          0,      0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,          0,          0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vyaw, 0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_vyaw, q_vyaw_vyaw,0,      0,
          0,      0,      0,      0,      0,      0,      0,          0,          q_r,    0,
          0,      0,      0,      0,      0,      0,      0,          0,          0,      q_d_zc;

    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_x_ = declare_parameter("ekf.r_x", 0.05);
  r_y_ = declare_parameter("ekf.r_y", 0.05);
  r_z_ = declare_parameter("ekf.r_z", 0.05);
  r_yaw_ = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z) {
    Eigen::Matrix<double, Z_N, Z_N> r;
    // clang-format off
    r << r_x_ * std::abs(z[0]), 0, 0, 0,
         0, r_y_ * std::abs(z[1]), 0, 0,
         0, 0, r_z_ * std::abs(z[2]), 0,
         0, 0, 0, r_yaw_;
    // clang-format on
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  tracker_->ekf = std::make_unique<RobotStateEKF>(f, h, u_q, u_r, p0);

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&ArmorSolverNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>("armor_solver/measurement",
                                                                         rclcpp::SensorDataQoS());

  // Publisher
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("armor_solver/target",
                                                                   rclcpp::SensorDataQoS());
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  // Enable/Disable Armor Solver
  enable_ = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode",
    std::bind(
      &ArmorSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (debug_mode_) {
    initMarkers();
  }

  // 反投影可视化初始化
  enable_reprojection_visualization_ = this->declare_parameter("enable_reprojection_visualization", true);
  if (enable_reprojection_visualization_) {
    // 初始化图像传输
    image_transport_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
    
    // 创建图像发布器
    reprojection_img_pub_ = image_transport_->advertise("armor_solver/reprojection_image", 1);
    
    // 订阅相机信息
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->cameraInfoCallback(msg);
      });
    
    // 订阅原始图像
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg);
      });
    
    FYT_INFO("armor_solver", "Reprojection visualization enabled!");
  }

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorSolverNode::timerCallback() {
  

  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  // Init message
  rm_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);
      //last_yaw=control_msg.yaw;
      //last_pitch=control_msg.pitch;
      //std::cout<<"control_msg distance: "<<control_msg.distance<<std::endl;
      //std::cout<<"last yaw: "<<last_yaw<<" last pitch: "<<last_pitch<<std::endl;
    } catch (...) {
      FYT_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.fire_advice = false;
    }
    gimbal_pub_->publish(control_msg);
  } else {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.fire_advice = false;
    //control_msg.yaw = last_yaw;
    //control_msg.pitch = last_pitch;
  }
  
  if (debug_mode_) {
    publishMarkers(armor_target_, control_msg);
  }
}

void ArmorSolverNode::initMarkers() noexcept {
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}

void ArmorSolverNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_msg) {
  
  //std::cout<<"start  solver"<<std::endl;
  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
    // 如果已经有相机信息，立即设置
    if (latest_camera_info_) {
      solver_->setCameraParameters(latest_camera_info_);
    }
  }

  // 实现装甲板位置从云台坐标系转换到世界坐标系的增强功能
  transformArmorsToWorldCoordinates(armors_msg);

  // Filter abnormal armors
  armors_msg->armors.erase(std::remove_if(armors_msg->armors.begin(),
                                          armors_msg->armors.end(),
                                          [](const rm_interfaces::msg::Armor &armor) {
                                            return abs(armor.pose.position.z) > 2;
                                          }),
                           armors_msg->armors.end());

  // Init message
  rm_interfaces::msg::Measurement measure_msg;
  rm_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;


  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thres_ / dt_));

    // 设置预测函数（前哨站和其他机器人使用相同的运动模型）
    tracker_->ekf->setPredictFunc(Predict{dt_, MotionModel::CONSTANT_VEL_ROT});
    tracker_->update(armors_msg);
    // Publish measurement
    measure_msg.x = tracker_->measurement(0);
    measure_msg.y = tracker_->measurement(1);
    measure_msg.z = tracker_->measurement(2);
    measure_msg.yaw = tracker_->measurement(3);
    measure_pub_->publish(measure_msg);
 
 

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING ||
               tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto &state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.d_zc = state(9);
      target_msg.d_za = tracker_->d_za;
    }
  }

  // Store and Publish the target_msg
  armor_target_ = target_msg;
  target_pub_->publish(target_msg);

  // 发布反投影可视化图像
  if (enable_reprojection_visualization_ && latest_image_ && latest_camera_info_ && solver_) {
    publishReprojectionImage(target_msg, armors_msg);
  }

  last_time_ = time;
}

void ArmorSolverNode::publishMarkers(const rm_interfaces::msg::Target &target_msg,
                                     const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header = target_msg.header;
  selection_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double d_za = target_msg.d_za, d_zc = target_msg.d_zc;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    // Draw armors
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = zc + d_zc +  (is_current_pair ? 0 : d_za);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = zc;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armors_marker_.id = i;
      armors_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
      armors_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armors_marker_);
    }

    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    selection_marker_.points.clear();
    selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.z = gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : solver_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }

  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

void ArmorSolverNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("armor_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      break;
    }
  }

  FYT_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

// 实现装甲板位置从云台坐标系转换到世界坐标系
void ArmorSolverNode::transformArmorsToWorldCoordinates(
    const rm_interfaces::msg::Armors::SharedPtr &armors_msg) {
  
  for (auto &armor : armors_msg->armors) {
    // Step 1: 准备姿态数据用于变换
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    
    try {
      // Step 2: 从相机坐标系变换到目标坐标系(通常是odom/world)
      geometry_msgs::msg::PoseStamped transformed_pose = tf2_buffer_->transform(ps, target_frame_);
      armor.pose = transformed_pose.pose;
      
      // Step 3: 获取云台到世界坐标系的变换 (用于后续处理)
      geometry_msgs::msg::TransformStamped gimbal_to_world_tf;
      try {
        gimbal_to_world_tf = tf2_buffer_->lookupTransform(
          target_frame_, "gimbal_link", armors_msg->header.stamp);
        
        // 验证变换结果 - 记录坐标变换信息
        if (debug_mode_) {
          FYT_DEBUG("armor_solver", "Armor {} transformed - World pos: x={:.3f}, y={:.3f}, z={:.3f}", 
                   armor.number,
                   armor.pose.position.x,
                   armor.pose.position.y,
                   armor.pose.position.z);
        }
        
      } catch (const tf2::TransformException &ex) {
        FYT_WARN("armor_solver", "Could not get gimbal to world transform: {}", ex.what());
      }
      
    } catch (const tf2::TransformException &ex) {
      FYT_ERROR("armor_solver", "Transform error from camera to {}: {}", target_frame_, ex.what());
      return;
    }
  }
}

// 相机信息回调函数
void ArmorSolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
  latest_camera_info_ = camera_info;
  
  // 将相机参数传递给solver
  if (solver_) {
    solver_->setCameraParameters(camera_info);
  }
  
  FYT_INFO("armor_solver", "Received camera_info: {}x{}, fx={:.2f}, fy={:.2f}", 
           camera_info->width, camera_info->height,
           camera_info->k[0], camera_info->k[4]);
}

// 图像回调函数
void ArmorSolverNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
  latest_image_ = image_msg;
}

// 发布反投影图像
void ArmorSolverNode::publishReprojectionImage(const rm_interfaces::msg::Target &target_msg,
                                               const rm_interfaces::msg::Armors::SharedPtr armors_ptr) {
  try {
    // 转换图像格式 - 支持RGB8和BGR8编码
    cv_bridge::CvImagePtr cv_ptr;
    if (latest_image_->encoding == sensor_msgs::image_encodings::RGB8) {
      cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::RGB8);
      // RGB转BGR用于OpenCV显示
      cv::Mat image_rgb = cv_ptr->image;
      cv::Mat image_bgr;
      cv::cvtColor(image_rgb, image_bgr, cv::COLOR_RGB2BGR);
      cv_ptr->image = image_bgr;
    } else {
      cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
    }
    cv::Mat image = cv_ptr->image.clone();
    
    // 获取反投影的装甲板点集
    if (target_msg.tracking && solver_) {
      auto reprojected_armors = solver_->reproject_all_armors(target_msg, tf2_buffer_);
      
      // 在图像上绘制反投影的装甲板
      drawReprojectedArmors(image, reprojected_armors, armors_ptr);
    }
    
    // 发布处理后的图像
    cv_bridge::CvImage out_msg;
    out_msg.header = latest_image_->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;
    
    reprojection_img_pub_.publish(out_msg.toImageMsg());
    
  } catch (cv_bridge::Exception& e) {
    FYT_ERROR("armor_solver", "cv_bridge exception: {}", e.what());
  }
}

// 绘制反投影装甲板
void ArmorSolverNode::drawReprojectedArmors(cv::Mat &image, 
                                           const std::vector<std::vector<cv::Point2f>> &reprojected_armors,
                                           const rm_interfaces::msg::Armors::SharedPtr armors_ptr) {
  
  // 绘制反投影的装甲板（绿色）
  for (size_t i = 0; i < reprojected_armors.size(); ++i) {
    const auto &armor_points = reprojected_armors[i];
    if (armor_points.size() >= 4) {
      // 绘制装甲板轮廓
      std::vector<cv::Point> contour;
      for (const auto &point : armor_points) {
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);
        // 检查点是否在图像范围内
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
          contour.push_back(cv::Point(x, y));
        }
      }
      
      if (contour.size() >= 4) {
        // 绘制装甲板边框
        cv::polylines(image, contour, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        
        // 绘制装甲板角点
        for (const auto &point : armor_points) {
          int x = static_cast<int>(point.x);
          int y = static_cast<int>(point.y);
          if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
          }
        }
        
        // 添加装甲板编号（如果有对应的检测结果）
        if (i < armors_ptr->armors.size()) {
          std::string armor_id = armors_ptr->armors[i].number;
          if (!armor_points.empty()) {
            int x = static_cast<int>(armor_points[0].x);
            int y = static_cast<int>(armor_points[0].y) - 10;
            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
              cv::putText(image, "Reproj_" + armor_id, cv::Point(x, y), 
                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            }
          }
        }
      }
    }
  }
  
  // 绘制检测到的装甲板（红色）
  // 使用armor的pose信息，将3D点投影到2D图像平面
  if (solver_ && latest_camera_info_) {
    // 获取相机参数（在循环外获取，避免重复计算）
    const auto &K = latest_camera_info_->k;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
      K[0], K[1], K[2],
      K[3], K[4], K[5],
      K[6], K[7], K[8]);
    
    const auto &D = latest_camera_info_->d;
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < D.size() && i < 5; ++i) {
      dist_coeffs.at<double>(0, i) = D[i];
    }
    
    // 定义3D模型点（在函数外部定义，避免重复）
    constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
    constexpr double SMALL_ARMOR_WIDTH = 135e-3;   // m
    constexpr double LIGHTBAR_LENGTH = 56e-3;      // m
    static const std::vector<cv::Point3f> BIG_POINTS{
      {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
      {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
    };
    static const std::vector<cv::Point3f> SMALL_POINTS{
      {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
      {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
    };
    
    for (const auto &armor : armors_ptr->armors) {
      try {
        // 重要：armor.pose已经在transformArmorsToWorldCoordinates中被转换到世界坐标系(odom)
        // 需要将其转换回相机坐标系才能进行投影
        geometry_msgs::msg::PoseStamped armor_pose_world;
        armor_pose_world.header = armors_ptr->header;
        armor_pose_world.header.frame_id = target_frame_;  // 当前在odom坐标系
        armor_pose_world.pose = armor.pose;
        
        // 转换回相机坐标系
        geometry_msgs::msg::PoseStamped armor_pose_camera;
        try {
          armor_pose_camera = tf2_buffer_->transform(armor_pose_world, "camera_link");
        } catch (const tf2::TransformException &ex) {
          FYT_WARN("armor_solver", "Failed to transform armor pose to camera frame: {}", ex.what());
          continue;
        }
        
        // 获取armor在相机坐标系中的3D位置和姿态
        Eigen::Vector3d armor_position(
          armor_pose_camera.pose.position.x,
          armor_pose_camera.pose.position.y,
          armor_pose_camera.pose.position.z
        );
        
        // 从四元数获取旋转矩阵（装甲板坐标系到相机坐标系）
        Eigen::Quaterniond armor_quaternion(
          armor_pose_camera.pose.orientation.w,
          armor_pose_camera.pose.orientation.x,
          armor_pose_camera.pose.orientation.y,
          armor_pose_camera.pose.orientation.z
        );
        Eigen::Matrix3d R_armor2camera = armor_quaternion.toRotationMatrix();
        
        // 选择装甲板3D模型点（根据armor类型）
        const std::vector<cv::Point3f> *object_points = nullptr;
        if (armor.type == "large" || armor.type == "big") {
          object_points = &BIG_POINTS;
        } else {
          object_points = &SMALL_POINTS;
        }
        
        // 将3D模型点从装甲板坐标系转换到相机坐标系
        // armor_pose_camera.pose.position是装甲板中心在相机坐标系中的位置
        // armor_pose_camera.pose.orientation是装甲板坐标系到相机坐标系的旋转
        std::vector<cv::Point3f> object_points_camera;
        for (const auto &pt : *object_points) {
          Eigen::Vector3d pt_armor(pt.x, pt.y, pt.z);
          // 将装甲板坐标系中的点转换到相机坐标系
          Eigen::Vector3d pt_camera = R_armor2camera * pt_armor + armor_position;
          object_points_camera.push_back(cv::Point3f(
            pt_camera.x(), pt_camera.y(), pt_camera.z()
          ));
        }
        
        // 投影到2D图像平面
        // 由于点已经在相机坐标系中，rvec和tvec都为零
        cv::Vec3d rvec(0, 0, 0);
        cv::Vec3d tvec(0, 0, 0);
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(object_points_camera, rvec, tvec, 
                         camera_matrix, dist_coeffs, image_points);
        
        // 绘制检测到的装甲板（红色）
        if (image_points.size() >= 4) {
          std::vector<cv::Point> contour;
          for (const auto &point : image_points) {
            int x = static_cast<int>(point.x);
            int y = static_cast<int>(point.y);
            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
              contour.push_back(cv::Point(x, y));
            }
          }
          
          if (contour.size() >= 4) {
            // 绘制装甲板边框（红色）
            cv::polylines(image, contour, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            
            // 绘制装甲板角点（红色）
            for (const auto &point : image_points) {
              int x = static_cast<int>(point.x);
              int y = static_cast<int>(point.y);
              if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
              }
            }
            
            // 绘制装甲板中心点
            cv::Point2f center(0, 0);
            for (const auto &point : image_points) {
              center += point;
            }
            center /= static_cast<float>(image_points.size());
            int cx = static_cast<int>(center.x);
            int cy = static_cast<int>(center.y);
            if (cx >= 0 && cx < image.cols && cy >= 0 && cy < image.rows) {
              cv::circle(image, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
            }
            
            // 添加装甲板编号
            if (!armor.number.empty()) {
              int x = static_cast<int>(center.x);
              int y = static_cast<int>(center.y) - 15;
              if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                cv::putText(image, "Det_" + armor.number, cv::Point(x, y), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
              }
            }
          }
        }
      } catch (const std::exception &e) {
        FYT_WARN("armor_solver", "Error drawing detected armor: {}", e.what());
        continue;
      }
    }
  }
  
  // 添加图例
  cv::Point legend_pos(10, 30);
  cv::putText(image, "Green: Reprojected", legend_pos, 
             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  
  legend_pos.y += 25;
  cv::putText(image, "Red: Detected", legend_pos, 
             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorSolverNode)
