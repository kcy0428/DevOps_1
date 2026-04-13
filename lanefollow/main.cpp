#include "rclcpp/rclcpp.hpp"
#include "lanefollow_real/lanefollow_real.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowProcessor>());
    rclcpp::shutdown();
    return 0;
}
