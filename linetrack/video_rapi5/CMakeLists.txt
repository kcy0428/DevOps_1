cmake_minimum_required(VERSION 3.16)
project(camera_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
#build
add_executable(pub src/pub.cpp src/main.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(pub ${OpenCV_LIBRARIES})

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(sub ${OpenCV_LIBRARIES})

# Install
install(TARGETS
  pub
  sub
  DESTINATION lib/${PROJECT_NAME})

ament_package() 