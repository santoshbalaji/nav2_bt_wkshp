cmake_minimum_required(VERSION 3.8)
project(turtlebot_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

add_executable(
  feature_detection src/feature_detection.cpp
)

ament_target_dependencies(
  feature_detection
  rclcpp
  sensor_msgs
  PCL
  pcl_ros
  pcl_conversions)

install(TARGETS
  feature_detection
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch world model rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
