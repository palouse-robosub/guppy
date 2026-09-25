cmake_minimum_required(VERSION 3.8)
project(guppy_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# installation

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeState.srv"
  "msg/State.msg"

  "msg/CanFrame.msg"
  "srv/SendCan.srv"

  "srv/SetHoldPose.srv"

  "msg/Toast.msg"

  "msg/CornerDetection.msg"
  "msg/CornerDetectionList.msg"
  "msg/TransformList.msg"

  "action/Navigate.action"

  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    vision_msgs
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
