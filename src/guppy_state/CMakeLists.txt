cmake_minimum_required(VERSION 3.10)
project(guppy_state)

# Default to C++17 for ROS2 Jazzy compatibility
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Enable warnings
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(guppy_msgs REQUIRED)

add_executable(state_manager src/state_manager.cpp)
add_executable(navstart src/navstart.cpp)
add_executable(led_pub src/led_pub.cpp)
ament_target_dependencies(led_pub rclcpp std_msgs guppy_msgs)
ament_target_dependencies(state_manager rclcpp std_srvs std_msgs geometry_msgs guppy_msgs)
ament_target_dependencies(navstart rclcpp std_msgs geometry_msgs guppy_msgs)

install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  state_manager # what goes here
  led_pub
  navstart
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
