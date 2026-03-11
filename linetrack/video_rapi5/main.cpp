#include "camera_ros2/pub.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::string video_path = "/home/rapi5/ros2_ws/simulation/5_lt_cw_100rpm_out.mp4";
    auto node = std::make_shared<CamPublisher>(video_path);
    node->run();
    rclcpp::shutdown();
    return 0;
} 