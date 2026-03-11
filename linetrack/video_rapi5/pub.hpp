#ifndef _PUB_HPP_
#define _PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CamPublisher : public rclcpp::Node
{
public:
    explicit CamPublisher(const std::string &video_path);
    void run();

private:
    std::string video_path_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::WallRate loop_rate_;
};

#endif