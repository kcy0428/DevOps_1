cmake_minimum_required(VERSION 3.8)
project(lanefollow_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lanefollow_real src/lanefollow_real.cpp src/main.cpp)
target_include_directories(lanefollow_real PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lanefollow_real PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  lanefollow_real
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "OpenCV"
)

install(TARGETS lanefollow_real
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
