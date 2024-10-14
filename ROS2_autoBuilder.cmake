# ROS2_autoBuilder.cmake
# Author: Doraemonjayo
# README: https://github.com/Doraemonjayo/ROS2_autoBuildCMake/
# This CMake file configures the build for the project based on the source directory structure.

# Get the project name from the source directory name
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

# Add compile options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the required standards for C++ and C
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set the include directories for building and installation
include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Function to build ROS2 nodes
function(ROS2_autoBuildNodes)
    # Find specified dependencies
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)
    endforeach()

    # Recursively search for node source files
    file(GLOB_RECURSE NODES
        ${CMAKE_SOURCE_DIR}/nodes/*.c
        ${CMAKE_SOURCE_DIR}/nodes/*.cpp
        ${CMAKE_SOURCE_DIR}/nodes/*.cc
        ${CMAKE_SOURCE_DIR}/nodes/*.cxx
        ${CMAKE_SOURCE_DIR}/nodes/*.C
        ${CMAKE_SOURCE_DIR}/nodes/*.cp
    )

    # Recursively search for library source files
    file(GLOB_RECURSE LIB_SOURCES
        ${CMAKE_SOURCE_DIR}/src/*.c
        ${CMAKE_SOURCE_DIR}/src/*.cpp
        ${CMAKE_SOURCE_DIR}/src/*.cc
        ${CMAKE_SOURCE_DIR}/src/*.cxx
        ${CMAKE_SOURCE_DIR}/src/*.C
        ${CMAKE_SOURCE_DIR}/src/*.cp
    )

    # Create executables for each node
    foreach(NODE ${NODES})
        get_filename_component(NODE_NAME ${NODE} NAME_WE)
        add_executable(${NODE_NAME} ${NODE} ${LIB_SOURCES})
        ament_target_dependencies(${NODE_NAME} ${ARGN})
        install(
            TARGETS ${NODE_NAME}
            DESTINATION lib/${PROJECT_NAME}
        )
    endforeach()
endfunction()

# Function to generate ROS2 message interfaces
function(ROS2_autoGenerateInterfaces)
    # Find specified dependencies
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)
    endforeach()

    # Recursively search for message, service, and action files
    file(GLOB_RECURSE MSGS ${CMAKE_SOURCE_DIR}/msg/*.msg)
    file(GLOB_RECURSE SRVS ${CMAKE_SOURCE_DIR}/srv/*.srv)
    file(GLOB_RECURSE ACTIONS ${CMAKE_SOURCE_DIR}/action/*.action)

    # Convert absolute paths to relative paths
    set(RELATIVE_MSGS)
    foreach(msg IN LISTS MSGS)
        file(RELATIVE_PATH relative_msg ${CMAKE_SOURCE_DIR} ${msg})
        list(APPEND RELATIVE_MSGS ${relative_msg})
    endforeach()

    set(RELATIVE_SRVS)
    foreach(srv IN LISTS SRVS)
        file(RELATIVE_PATH relative_srv ${CMAKE_SOURCE_DIR} ${srv})
        list(APPEND RELATIVE_SRVS ${relative_srv})
    endforeach()

    set(RELATIVE_ACTIONS)
    foreach(action IN LISTS ACTIONS)
        file(RELATIVE_PATH relative_action ${CMAKE_SOURCE_DIR} ${action})
        list(APPEND RELATIVE_ACTIONS ${relative_action})
    endforeach()

    if(RELATIVE_MSGS OR RELATIVE_SRVS OR RELATIVE_ACTIONS)  # At least one of the file types exists
        # Generate interfaces for messages, services, and actions
        rosidl_generate_interfaces(${PROJECT_NAME}
            ${RELATIVE_MSGS}
            ${RELATIVE_SRVS}
            ${RELATIVE_ACTIONS}
            DEPENDENCIES ${ARGN}
        )

        # Export runtime dependencies
        ament_export_dependencies(rosidl_default_runtime)
    endif()
endfunction()
