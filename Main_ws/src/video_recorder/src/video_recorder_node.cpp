#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <video_recorder/video_recorder_node.hpp>

// namespace fyt::auto_aim {
VideoRecorderNode::VideoRecorderNode(const rclcpp::NodeOptions &options)
    : Node("video_recorder", options) {

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "result_img", rclcpp::SensorDataQoS(),
      std::bind(&VideoRecorderNode::imageCallback, this,
                std::placeholders::_1));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

//   heartbeat_ = HeartBeatPublisher::create(this);
}

void VideoRecorderNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
        try
        {
            // 转换为 OpenCV 格式
            cv::Mat frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;

            if (frame.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty image!");
                return;
            }

            // 第一帧时初始化 VideoWriter
            if (writer_.isOpened())
            {
                int width = frame.cols;
                int height = frame.rows;
                writer_ = cv::VideoWriter("output.avi",
                                           cv::VideoWriter::fourcc('X','V','I','D'),
                                           20.0,
                                           cv::Size(width, height));
                RCLCPP_INFO(this->get_logger(), "Started recording video with resolution %dx%d.", width, height);
            }

            // 写入帧
            writer_.write(frame);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
}

// } // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(VideoRecorderNode)
