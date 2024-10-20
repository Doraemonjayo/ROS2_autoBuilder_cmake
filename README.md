# ROS2_autoBuilder.cmake 日本語は[こちら](https://github.com/Doraemonjayo/ROS2_autoBuilder_cmake/blob/main/README_jp.md)


## Overview
`ROS2_autoBuilder.cmake` is a CMake script designed to automate the build configuration for ROS 2 projects. This CMake file simplifies the process of building nodes and libraries based on a specified directory structure, as well as generating custom message, service, and action interfaces. It automatically searches for all `.cpp` files and other necessary files within the specified directories, making it unnecessary to manually modify the CMakeLists.txt file for every change.

## Usage

### Basic CMakeLists.txt Example

```cmake
cmake_minimum_required(VERSION 3.16.1)

# Get the project name from the current folder name
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

# Include ROS2_autoBuilder.cmake
include(ROS2_autoBuilder.cmake)

# Specify C++ and C standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# Automatically generate custom interfaces (messages, services, actions)
ROS2_autoGenerateInterfaces(std_msgs)

# Automatically build nodes
ROS2_autoBuildNodes(rclcpp std_msgs)

# Set up the ament package
ament_package()
```

In this CMakeLists.txt file, the `ROS2_autoGenerateInterfaces` and `ROS2_autoBuildNodes` functions are used to automatically generate custom interfaces and build nodes. By simply changing the dependencies, you can easily adapt to various packages, allowing for better reusability across different projects.

### Directory Structure

The following directory structure is recommended, but you may omit directories that do not contain any files.

```
your_project/
├── CMakeLists.txt
├── ROS2_autoBuilder.cmake
├── nodes/
│   ├── node1.cpp
│   └── node2.cpp
├── src/
│   ├── lib1.cpp
│   └── lib2.cpp
├── include/
│   ├── lib1.hpp
│   └── lib2.hpp
├── msg/
│   └── MyMsg.msg
├── srv/
│   └── MySrv.srv
└── action/
    └── MyAction.action
```

- **nodes/**: Each file in this directory contains a `main` function and is built as a standalone executable ROS 2 node. The source files located here will function independently as separate nodes.
- **src/**: This directory is for placing source files for libraries and other supportive functions. These source files will be linked to all the node source files during the build process, allowing for shared functionality or utility functions.
- **include/**: This directory contains header files that are dependent on the libraries or nodes.
- **msg/**: This directory is for custom message files, allowing you to define specific data structures used in ROS 2 communication.
- **srv/**: This directory is for custom service files, defining services that allow specific data to be exchanged in requests and responses.
- **action/**: This directory is for custom action files, allowing you to define goals, feedback, and results for action communications.

### Automatic Interface Generation

By using the `ROS2_autoGenerateInterfaces()` function, you can easily generate message, service, and action files automatically. Here’s an example of specifying `std_msgs` as a dependency and generating custom interfaces:

```cmake
ROS2_autoGenerateInterfaces(std_msgs)
```

### Automatic Node Building

The `ROS2_autoBuildNodes()` function is used to automatically build nodes based on the specified dependency libraries. Here’s an example specifying `rclcpp` and `std_msgs` as dependency libraries:

```cmake
ROS2_autoBuildNodes(rclcpp std_msgs)
```

### License
This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
