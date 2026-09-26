cmake_minimum_required(VERSION 3.10)
project(guppy_can)

# Default to C++17 for ROS2 Jazzy compatibility
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Enable warnings
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guppy_msgs REQUIRED)

# CAN RX publisher executable
add_executable(can_rx src/can_rx.cpp)
ament_target_dependencies(can_rx rclcpp guppy_msgs)

# CAN TX service executable
add_executable(can_tx src/can_tx.cpp)
ament_target_dependencies(can_tx rclcpp guppy_msgs)

install(TARGETS
        can_rx
        can_tx

        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)
ament_package()