#ifndef VIDEO_RECORDER_NODE_HPP_
#define VIDEO_RECORDER_NODE_HPP_

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// namespace fyt::auto_aim {

class VideoRecorderNode : public rclcpp::Node {
public:
  VideoRecorderNode(const rclcpp::NodeOptions &options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  std::string video_filename_;
  // Heartbeat
//   HeartBeatPublisher::SharedPtr heartbeat_;

  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  cv::VideoWriter writer_;
};

// }

#endif //VIDEO_RECORDER_NODE_HPP_