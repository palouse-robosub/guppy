cmake_minimum_required(VERSION 3.8)
project(guppy_nav)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(control_toolbox REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# targets

add_library(pose_setter SHARED
  src/pose_setter_server.cpp
)

ament_target_dependencies(pose_setter
    rclcpp
    rclcpp_action
    rclcpp_components
    guppy_msgs
    nav_msgs
    geometry_msgs
    Eigen3
    control_toolbox
)

target_compile_definitions(pose_setter
  PRIVATE
    "GUPPY_NAV_BUILDING_DLL"
)

rclcpp_components_register_node(pose_setter
    PLUGIN "PoseSetterServer"
    EXECUTABLE pose_setter_server
)

target_include_directories(pose_setter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(pose_setter
  guppy_util::guppy_util
)


add_executable(behavior_tree
  src/behavior_tree.cpp
  src/acquire_detection.cpp
  src/change_state_behavior.cpp
  src/face_detection_behavior.cpp
  src/move_toward_behavior.cpp
  src/pose_setter_behavior.cpp
)

ament_target_dependencies(behavior_tree
    rclcpp
    guppy_msgs
    behaviortree_cpp
    behaviortree_ros2
)

target_include_directories(behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(behavior_tree
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  guppy_util::guppy_util
)

# installation

install(
  TARGETS
    pose_setter
    pose_setter_server
    behavior_tree
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
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
