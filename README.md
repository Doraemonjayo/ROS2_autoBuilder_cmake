# ROS2_autoBuilder.cmake　日本語は[こちら](https://github.com/Doraemonjayo/ROS2_autoBuilder_cmake/blob/main/README_jp.md)

`ROS2_autoBuilder.cmake` is a CMake file designed to automate the build of ROS2 projects based on the structure of the source directory. By using this CMake file, you can easily handle node builds as well as the generation of message, service, and action interfaces.

## Features
- **Automatic Node Build**: Automatically builds all node source files (`.c`, `.cpp`, etc.) in the `nodes` directory.
- **Library Integration**: Links all library sources found in the `src` directory to each node.
- **Interface Generation for Messages, Services, and Actions**: Automatically generates interfaces from files in the `msg`, `srv`, and `action` directories.
- **Automatic Dependency Detection**: Automatically searches for required packages and sets up dependencies.

## Usage

1. Include this CMake file in your `CMakeLists.txt`. 

```cmake
include(ROS2_autoBuilder.cmake)
```

2. To build nodes, use the `ROS2_autoBuildNodes()` function, passing the required packages as arguments.

```cmake
ROS2_autoBuildNodes(<package1> <package2> ...)
```

3. To generate message, service, and action interfaces, use the `ROS2_autoGenerateInterfaces()` function, passing the required packages as arguments.

```cmake
ROS2_autoGenerateInterfaces(<package1> <package2> ...)
```

## Specific Directory Structure and File Examples
This CMake file works with the following directory structure and file layout. If there are no files in a folder, that folder can be omitted.

```
.
├── CMakeLists.txt                      # CMake configuration file for the project
├── ROS2_autoBuilder.cmake              # CMake auto-builder configuration for this project
├── nodes/                              # Directory for node source files
│   ├── node1.cpp                       # Node 1 (includes the main function)
│   └── node2.cpp                       # Node 2 (includes the main function)
├── src/                                # Directory for common library sources
│   ├── common_functions.cpp            # Library source file (common between nodes)
│   └── helper.cpp                      # File for helper functions
├── include/                            # Directory for header files
│   ├── common_functions.hpp            # Header file for library
│   └── helper.hpp                      # Header file for helper functions
├── msg/                                # Directory for message definition files
│   └── ExampleMessage.msg              # Message definition file
├── srv/                                # Directory for service definition files
│   └── ExampleService.srv              # Service definition file
└── action/                             # Directory for action definition files
    └── ExampleAction.action            # Action definition file
```

### `nodes/`
The `nodes` directory contains **source files for executable nodes**. These files include the `main` function and operate as **independent ROS2 nodes**.

- Example: `node1.cpp`
  ```cpp
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/string.hpp"

  int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("node1");
      RCLCPP_INFO(node->get_logger(), "Hello from Node 1!");
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
  }
  ```

- Example: `node2.cpp`
  ```cpp
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/string.hpp"

  int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("node2");
      RCLCPP_INFO(node->get_logger(), "Hello from Node 2!");
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
  }
  ```

### `src/`
The `src` directory contains library sources that are shared among nodes. Source files in this directory are linked to **all nodes** being built.

- Example: `common_functions.cpp`
  ```cpp
  #include "common_functions.hpp"

  void example_function() {
      // Common processing
  }
  ```

- Example: `helper.cpp`
  ```cpp
  #include "helper.hpp"

  void helper_function() {
      // Implementation of helper functions
  }
  ```

### `include/`
The `include` directory contains header files that are included by the library sources and nodes.

- Example: `common_functions.hpp`
  ```cpp
  void example_function();
  ```

- Example: `helper.hpp`
  ```cpp
  void helper_function();
  ```

### `msg/`, `srv/`, `action/`
These directories are for storing ROS2 message, service, and action definition files. If there are no files in these folders, they can be omitted.

- Example: `ExampleMessage.msg`
  ```
  string data
  ```

- Example: `ExampleService.srv`
  ```
  int32 a
  int32 b
  ---
  int32 sum
  ```

- Example: `ExampleAction.action`
  ```
  # Goal
  int32 order

  ---
  # Result
  string result

  ---
  # Feedback
  string feedback
  ```

## Example of CMakeLists.txt
Here is a specific example of `CMakeLists.txt`. This file can be used across different ROS2 packages by only changing the dependency package names. The project name is automatically set based on the directory name.

```cmake
cmake_minimum_required(VERSION 3.16.3)

# Get the project name from the source directory name
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

include(ROS2_autoBuilder.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# Specify node dependencies for building
ROS2_autoBuildNodes(rclcpp std_msgs)

# Specify message interface dependencies for generation
ROS2_autoGenerateInterfaces(std_msgs)

ament_package()
```

This `CMakeLists.txt` can be used across different packages simply by changing the required package dependencies.

## Note
- Place executable files containing the `main` function in `nodes/`.
- Place common libraries linked to all nodes in the `src/` directory.
- If there are no files in `msg/`, `srv/`, or `action/` directories, those folders can be omitted.

## License
This project is licensed under the [MIT License](LICENSE).
