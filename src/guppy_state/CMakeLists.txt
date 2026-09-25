cmake_minimum_required(VERSION 3.8)
project(guppy_state)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
)

# dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# executables

add_executable(state_manager
  src/state_manager.cpp
)

ament_target_dependencies(state_manager
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
  guppy_msgs
)

target_link_libraries(state_manager
  guppy_util::guppy_util
)


add_executable(nav_start
  src/nav_start.cpp
)

ament_target_dependencies(nav_start
  rclcpp
  std_msgs
  geometry_msgs
  guppy_msgs
)

target_link_libraries(nav_start
  guppy_util::guppy_util
)


add_executable(led_pub
  src/led_pub.cpp
)

ament_target_dependencies(led_pub
  rclcpp
  std_msgs
  guppy_msgs
)

target_link_libraries(led_pub
  guppy_util::guppy_util
)

# installation

install(
  TARGETS
    state_manager
    led_pub
    nav_start
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
