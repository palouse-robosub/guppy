cmake_minimum_required(VERSION 3.8)
project(guppy_can)

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
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# targets

add_executable(can_rx
  src/can_rx.cpp
)

ament_target_dependencies(can_rx
  rclcpp
  guppy_msgs
)

target_link_libraries(can_rx
  guppy_util::guppy_util
)


add_executable(can_tx
  src/can_tx.cpp
)

ament_target_dependencies(can_tx
  rclcpp
  guppy_msgs
)

target_link_libraries(can_tx
  guppy_util::guppy_util
)

# installation

install(
  TARGETS
    can_rx
    can_tx
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
