#ifndef _SUB_HPP_
#define _SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using std::placeholders::_1;

#define STRAIGHT_RPM  100.0
#define GAIN_K        1.0

class LineDetector : public rclcpp::Node {
private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void key_timer_callback();
    static bool kbhit();
    static int getch();

    bool first_run_;
    cv::Point tmp_pt_;
    bool running_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr key_timer_;

public:
    LineDetector();
};

#endif