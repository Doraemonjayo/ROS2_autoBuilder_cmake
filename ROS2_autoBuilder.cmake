# ROS2_autoBuilder.cmake
# Author: Doraemonjayo
# README: https://github.com/Doraemonjayo/ROS2_autoBuilder_cmake
# This CMake file configures the build for the project based on the source directory structure.

# Add compile options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Enable all compiler warnings
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the required standards for C++ and C
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set include directories for building and installation
include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # For building from source
    $<INSTALL_INTERFACE:include>  # For installation paths
)

# Find necessary packages
find_package(ament_cmake REQUIRED)  # Required for CMake integration with ROS 2
find_package(rosidl_default_generators REQUIRED)  # Required for generating ROS 2 message interfaces

# Recursively search for node source files
file(GLOB_RECURSE R2AB_NODES_SOURCES
    ${CMAKE_SOURCE_DIR}/nodes/**/*.c
    ${CMAKE_SOURCE_DIR}/nodes/**/*.cpp
    ${CMAKE_SOURCE_DIR}/nodes/**/*.c++
    ${CMAKE_SOURCE_DIR}/nodes/**/*.cc
    ${CMAKE_SOURCE_DIR}/nodes/**/*.cxx
    ${CMAKE_SOURCE_DIR}/nodes/**/*.C
    ${CMAKE_SOURCE_DIR}/nodes/**/*.cp
)

# Initialize a list to store node executable names
set(R2AB_NODES)
foreach(node_file IN LISTS R2AB_NODES_SOURCES)
    # Extract the base name from the file path
    get_filename_component(temp_node ${node_file} NAME_WE)
    # Create an executable for each node source file
    add_executable(${temp_node} ${node_file})
    # Append the node name to the list of nodes
    list(APPEND R2AB_NODES ${temp_node})
endforeach()

# Recursively search for library source files
file(GLOB_RECURSE R2AB_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/src/**/*.c
    ${CMAKE_SOURCE_DIR}/src/**/*.cpp
    ${CMAKE_SOURCE_DIR}/src/**/*.c++
    ${CMAKE_SOURCE_DIR}/src/**/*.cc
    ${CMAKE_SOURCE_DIR}/src/**/*.cxx
    ${CMAKE_SOURCE_DIR}/src/**/*.C
    ${CMAKE_SOURCE_DIR}/src/**/*.cp
)

# Add a library if any source files were found
if(R2AB_LIB_SOURCES)
    add_library(R2AB_LIB ${R2AB_LIB_SOURCES})  # Create a library from the source files
endif()

# Link the library to each node if it exists
foreach(node IN LISTS R2AB_NODES)
    if(TARGET R2AB_LIB)
        target_link_libraries(${node} ${R2AB_LIB})  # Link the library to the node
    endif()
endforeach()

# Recursively search for message, service, and action files
file(GLOB_RECURSE R2AB_ABSOLUTE_INTERFACES
    ${CMAKE_SOURCE_DIR}/msg/**/*.msg
    ${CMAKE_SOURCE_DIR}/srv/**/*.srv
    ${CMAKE_SOURCE_DIR}/action/**/*.action
)

# Initialize a list to store relative interface file paths
set(R2AB_INTERFACES)
foreach(absolute_interface IN LISTS R2AB_ABSOLUTE_INTERFACES)
    # Convert absolute paths to relative paths
    file(RELATIVE_PATH interface ${CMAKE_SOURCE_DIR} ${absolute_interface})
    list(APPEND R2AB_INTERFACES ${interface})  # Append the relative path to the interfaces list
endforeach()

# Function to build ROS2 nodes
function(ROS2_autoBuildNodes)
    # Find specified dependencies
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)  # Ensure required packages are found
    endforeach()

    # Create executables for each node
    foreach(node IN LISTS R2AB_NODES)
        ament_target_dependencies(${node} ${ARGN})  # Add dependencies to the node
        install(
            TARGETS ${node}  # Specify the target for installation
            DESTINATION lib/${PROJECT_NAME}  # Installation destination
        )
    endforeach()
endfunction()

# Function to generate ROS2 message interfaces
function(ROS2_autoGenerateInterfaces)
    # Find specified dependencies
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)  # Ensure required packages are found
    endforeach()

    # Generate interfaces if any are found
    if(R2AB_INTERFACES)
        rosidl_generate_interfaces(${PROJECT_NAME}
            ${R2AB_INTERFACES}  # List of interfaces to generate
            DEPENDENCIES ${ARGN}  # List of dependencies
        )

        # Get the type support target for the generated interfaces
        rosidl_get_typesupport_target(my_typesupport_cpp
            ${PROJECT_NAME}
            rosidl_typesupport_cpp
        )
        
        # Link the type support to each node
        foreach(node IN LISTS R2AB_NODES)
            target_link_libraries(${node}
                ${my_typesupport_cpp}  # Link type support libraries
            )
        endforeach()

        # Export runtime dependencies
        ament_export_dependencies(rosidl_default_runtime)  # Ensure runtime dependencies are available
    endif()
endfunction()
