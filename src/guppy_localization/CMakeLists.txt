cmake_minimum_required(VERSION 3.5)
project(guppy_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# targets

add_executable(barometer_pub
  src/barometer_pub.cpp
)

ament_target_dependencies(barometer_pub
  rclcpp
  sensor_msgs
  geometry_msgs
  guppy_msgs
  std_msgs
)

target_link_libraries(barometer_pub
  guppy_util::guppy_util
)


add_executable(combine_sensors
  src/combine.cpp
)

ament_target_dependencies(combine_sensors
  tf2
  tf2_geometry_msgs
  tf2_ros rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  Eigen3
)

target_link_libraries(combine_sensors
  guppy_util::guppy_util
)

# installation

install(
  TARGETS
    barometer_pub
    combine_sensors
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# package

ament_package()
