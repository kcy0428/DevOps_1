#include "camera_ros2/pub.hpp"

CamPublisher::CamPublisher(const std::string &video_path)
: Node("campub_video"),
  video_path_(video_path),
  loop_rate_(30.0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile);

    cap_.open(video_path_, cv::CAP_FFMPEG);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
    }
}
 
void CamPublisher::run()
{
    cv::Mat frame;
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;

    while (rclcpp::ok()) {
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame empty");
            break;
        }

        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        publisher_->publish(*msg);

        loop_rate_.sleep();
    }
}