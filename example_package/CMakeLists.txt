cmake_minimum_required(VERSION 3.8)

include(ROS2_autoBuilder.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

ROS2_autoBuildNodes(rclcpp std_msgs)
ROS2_autoGenerateInterfaces(rclcpp std_msgs)

ament_package()
