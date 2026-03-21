// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
#include <vector>
// ros2
#include <tf2_ros/buffer.h>
#include <angles/angles.h>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"

namespace fyt::auto_aim {

// 装甲板类型枚举
enum class ArmorType {
  SMALL = 0,
  BIG = 1
};
// Solver class used to solve the gimbal command from tracked target
class Solver {
public:
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  // explicit Solver(std::string trajectory_compensator_type, float max_tracking_v_yaw);
  ~Solver() = default;

  // Solve the gimbal command from tracked target
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

  std::vector<std::pair<double, double>> getTrajectory() const noexcept;
  void set_R_gimbal2world(const Eigen::Quaterniond & q);
  
  // 反投影功能 (已注释)
  // double outpost_reprojection_error(const std::vector<cv::Point2f> &armor_points,
  //                                  ArmorType type,
  //                                  const double & pitch,
  //                                  const Eigen::Vector3d &xyz_in_world) const noexcept;
  //
  // // 新增反投影功能
  // std::vector<cv::Point2f> reproject_single_armor(const Eigen::Vector3d &xyz_in_world,
  //                                                 double yaw,
  //                                                 double pitch,
  //                                                 ArmorType type,
  //                                                 const std::string &target_frame,
  //                                                 std::shared_ptr<tf2_ros::Buffer> tf2_buffer) const noexcept;
  //
  // std::vector<std::vector<cv::Point2f>> reproject_all_armors(const rm_interfaces::msg::Target &target,
  //                                                           std::shared_ptr<tf2_ros::Buffer> tf2_buffer) const noexcept;
  //
  // double calculate_reprojection_error(const std::vector<cv::Point2f> &detected_points,
  //                                    const std::vector<cv::Point2f> &reprojected_points) const noexcept;
  //
  // // 设置相机参数（从camera_info消息）
  // void setCameraParameters(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  //定义一个储存装甲板xyz和yaw的结构体
  struct ArmorData {

      float x;
      float y; 
      float z;
      float yaw;
  
    };
  
  //定义一个vector储存所有装甲板的数据
    std::vector<ArmorData> armors_data;
  int best_idx =-1;

private:
  // // Get the armor positions from the target robot
  // std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
  //                                                const double yaw,
  //                                                const double r1,
  //                                                const double r2,
  //                                                const double d_zc,
  //                                                const double d_za,
  //                                                const size_t armors_num) const noexcept;

  // Select the best armor to shoot
  // Return: selected idx in {0, 1, ..., armors_num - 1}
  int selectBestArmor(const std::vector<ArmorData>& armors_data) const noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  std::array<double, 3> rpy_;

  double prediction_delay_;
  double controller_delay_;

  double shooting_range_w_;
  double shooting_range_h_;
  double Relative_yaw_angle_deviation_;
  double Relative_yaw_angle_deviation_outpost_;
  double angular_velocity_coefficient_;

  double max_tracking_v_yaw_;
  int overflow_count_;
  int transfer_thresh_;

  double side_angle_;
  double min_switching_v_yaw_;

  // // 变换所需矩阵
  // Eigen::Matrix3d R_camera2gimbal_;
  // Eigen::Matrix3d R_gimbal2world_;
  // Eigen::Vector3d t_camera2gimbal_;
  
  // 相机参数 (用于反投影) (已注释)
  // cv::Mat camera_matrix_;
  // cv::Mat distort_coeffs_;

  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace fyt::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_
