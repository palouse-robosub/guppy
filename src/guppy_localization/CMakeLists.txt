cmake_minimum_required(VERSION 3.10)
project(guppy_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2 REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)


add_executable(barometer_pub src/barometer_pub.cpp)
add_executable(combine_sensors src/combine.cpp)
ament_target_dependencies(barometer_pub rclcpp sensor_msgs geometry_msgs guppy_msgs std_msgs)
ament_target_dependencies(combine_sensors tf2 tf2_geometry_msgs tf2_ros rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs Eigen3)

install(TARGETS
  barometer_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  combine_sensors
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()