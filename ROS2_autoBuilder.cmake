# ROS2_autoBuilder.cmake
# Author: Doraemonjayo
# README: https://github.com/Doraemonjayo/ROS2_autoBuilder_cmake
# This CMake file configures the build for the project based on the source directory structure.

# Add compiler warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ and C standard requirements
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set include directories for the project
set(R2AB_INCLUDE_DIR
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Recursively gather node source files into a list
file(GLOB_RECURSE R2AB_NODES_SOURCES
    ${CMAKE_SOURCE_DIR}/nodes/*.c
    ${CMAKE_SOURCE_DIR}/nodes/*.cpp
    ${CMAKE_SOURCE_DIR}/nodes/*.c++
    ${CMAKE_SOURCE_DIR}/nodes/*.cc
    ${CMAKE_SOURCE_DIR}/nodes/*.cxx
    ${CMAKE_SOURCE_DIR}/nodes/*.C
    ${CMAKE_SOURCE_DIR}/nodes/*.cp
)

# Recursively gather library source files into a list
file(GLOB_RECURSE R2AB_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.c
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/*.c++
    ${CMAKE_SOURCE_DIR}/src/*.cc
    ${CMAKE_SOURCE_DIR}/src/*.cxx
    ${CMAKE_SOURCE_DIR}/src/*.C
    ${CMAKE_SOURCE_DIR}/src/*.cp
)

# Initialize the list of ROS 2 interfaces (message, service, action files)
set(R2AB_INTERFACES)

# Recursively gather interface files (message, service, action) and store them with relative paths
file(GLOB_RECURSE R2AB_ABSOLUTE_INTERFACES
    ${CMAKE_SOURCE_DIR}/msg/*.msg
    ${CMAKE_SOURCE_DIR}/srv/*.srv
    ${CMAKE_SOURCE_DIR}/action/*.action
)

# Convert absolute paths to relative paths and append to the interface list
foreach(absolute_interface IN LISTS R2AB_ABSOLUTE_INTERFACES)
    file(RELATIVE_PATH interface ${CMAKE_SOURCE_DIR} ${absolute_interface})
    list(APPEND R2AB_INTERFACES ${interface})
endforeach()

# Flag to track if interfaces were generated
set(R2AB_INTERFACES_GENERATED False)

# Function to auto-generate interfaces (messages, services, actions) and their dependencies
function(ROS2_autoGenerateInterfaces)
    # Search for and include any dependency packages specified
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)
    endforeach()

    # If any interfaces exist, generate them
    if(R2AB_INTERFACES)
        rosidl_generate_interfaces(${PROJECT_NAME}
            ${R2AB_INTERFACES}
            DEPENDENCIES ${ARGN}
        )
        # Export dependencies for the generated interfaces
        ament_export_dependencies(rosidl_default_runtime)
        set(R2AB_INTERFACES_GENERATED True PARENT_SCOPE)  # Set flag to true in the parent scope
    endif()
endfunction()

# Function to automatically build nodes (executables)
function(ROS2_autoBuildNodes)
    # If interfaces were generated, get the typesupport target
    if(${R2AB_INTERFACES_GENERATED})
        rosidl_get_typesupport_target(my_typesupport_cpp
            ${PROJECT_NAME}
            rosidl_typesupport_cpp
        )
    endif()

    # Search for and include any dependency packages specified
    foreach(dependency IN LISTS ARGN)
        find_package(${dependency} REQUIRED)
    endforeach()

    # Loop over each node source file and create an executable for each
    foreach(node_source IN LISTS R2AB_NODES_SOURCES)
        get_filename_component(node ${node_source} NAME_WE)  # Get the base name of the node
        add_executable(${node} ${node_source} ${R2AB_LIB_SOURCES})  # Add executable for the node
        target_include_directories(${node} PUBLIC ${R2AB_INCLUDE_DIR})  # Set include directories
        ament_target_dependencies(${node} ${ARGN})  # Set dependencies for the node
        if(${R2AB_INTERFACES_GENERATED})
            target_link_libraries(${node} ${my_typesupport_cpp})  # Link typesupport if interfaces were generated
        endif()
        # Install the built node to the appropriate location
        install(
            TARGETS ${node}
            DESTINATION lib/${PROJECT_NAME}
        )
    endforeach()
endfunction()
