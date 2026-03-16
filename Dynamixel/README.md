
rapi5@rapi5:~/ros2_ws$ colcon build --symlink-install --packages-select dxl_rapi5
Starting >>> dxl_rapi5
--- stderr: dxl_rapi5
CMake Error at CMakeLists.txt:18 (find_package):
  By not providing "Finddynamixel_sdk.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "dynamixel_sdk", but CMake did not find one.

  Could not find a package configuration file provided by "dynamixel_sdk"
  with any of the following names:

    dynamixel_sdkConfig.cmake
    dynamixel_sdk-config.cmake

  Add the installation prefix of "dynamixel_sdk" to CMAKE_PREFIX_PATH or set
  "dynamixel_sdk_DIR" to a directory containing one of the above files.  If
  "dynamixel_sdk" provides a separate development package or SDK, be sure it
  has been installed.


gmake: *** [Makefile:294: cmake_check_build_system] Error 1
---
Failed   <<< dxl_rapi5 [0.82s, exited with code 2]

Summary: 0 packages finished [1.04s]
  1 package failed: dxl_rapi5
  1 package had stderr output: dxl_rapi5
rapi5@rapi5:~/ros2_ws$ colcon build --symlink-install --packages-select video_rapi5
Starting >>> video_rapi5
--- stderr: video_rapi5
CMake Error at /opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package_xml.cmake:53 (message):
  ament_package_xml() package name 'video_rapi5' in '/package.xml' does not
  match current PROJECT_NAME 'camera_ros2'.  You must call project() with the
  same package name before.
Call Stack (most recent call first):
  /opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake:63 (ament_package_xml)
  CMakeLists.txt:35 (ament_package)


---
Failed   <<< video_rapi5 [1.88s, exited with code 1]

Summary: 0 packages finished [2.11s]
  1 package failed: video_rapi5
  1 package had stderr output: video_rapi5
rapi5@rapi5:~/ros2_ws$ colcon build --symlink-install --packages-select video_rapi5
[0.250s] WARNING:colcon.colcon_core.package_selection:ignoring unknown package 'video_rapi5' in --packages-select

Summary: 0 packages finished [0.21s]
