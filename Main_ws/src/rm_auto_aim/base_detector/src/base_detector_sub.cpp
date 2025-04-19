#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"



class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("base_detector_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        cv::namedWindow("object", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("thresh", cv::WINDOW_AUTOSIZE);
        // cv::namedWindow("thresh", cv::WINDOW_AUTOSIZE);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving video frame");

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        detect_objects(cv_ptr->image);
    }

    void detect_objects(cv::Mat &image)
    {
        cv::Mat gray_img, thresh_img;
        cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
        cv::Mat dst_img;
        cv::medianBlur(gray_img, dst_img, 7);
        cv::threshold(gray_img, binary_img, 200, 255, cv::THRESH_BINARY);
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(dst_img, circles, cv::HOUGH_GRADIENT, 1, 30, 35, 100, 0, 1000);
        for (size_t i = 0; i < circles.size(); ++i) {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            int radius = c[2];
            cv::circle(image, center, radius, cv::Scalar(255, 0, 0), 10);
            cv::circle(image, center, 10, cv::Scalar(255, 0, 0), -1);
        }




        // std::vector<std::vector<cv::Point>> contours;
        // std::vector<cv::Vec4i> hierarchy;
        // cv::findContours(thresh_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
       
        // for (const auto &cnt : contours)
        // {
        //     if (cnt.size() < 100)
        //         continue;

        //     cv::Rect rect = cv::boundingRect(cnt);
        //     cv::drawContours(image, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 0, 255), 2);
        //     cv::circle(image,
        //                cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2),
        //                5, cv::Scalar(0, 255, 0), -1);
        // }
        // cv::imshow("thresh",thresh_img);
        cv::imshow("object", image);
        cv::imshow("thresh", binary_img);
  
        cv::waitKey(10);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
