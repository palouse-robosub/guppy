cmake_minimum_required(VERSION 3.10)
project(guppy_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(guppy_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(control_toolbox REQUIRED)

find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

#add_library(navigate_action SHARED src/navigate_action_server.cpp)
add_library(pose_setter SHARED src/pose_setter_server.cpp)
add_executable(behavior_tree src/behavior_tree.cpp)

#target_include_directories(navigate_action_server PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#)
target_include_directories(pose_setter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#target_compile_definitions(navigate_action_server
#    PRIVATE "GUPPY_NAV_BUILDING_DLL"
#)
target_compile_definitions(pose_setter
    PRIVATE "GUPPY_NAV_BUILDING_DLL"
)

#ament_target_dependencies(navigate_action
#    rclcpp rclcpp_action rclcpp_components guppy_msgs nav_msgs geometry_msgs Eigen3 control_toolbox
#)
ament_target_dependencies(pose_setter
    rclcpp rclcpp_action rclcpp_components guppy_msgs nav_msgs geometry_msgs Eigen3 control_toolbox
)
ament_target_dependencies(behavior_tree
    rclcpp guppy_msgs behaviortree_cpp behaviortree_ros2
)

target_link_libraries(behavior_tree
    behaviortree_cpp::behaviortree_cpp behaviortree_ros2::behaviortree_ros2
)

#rclcpp_components_register_node(navigate_action
#    PLUGIN "NavigateActionServer"
#    EXECUTABLE navigate_action_server
#)
rclcpp_components_register_node(pose_setter
    PLUGIN "PoseSetterServer"
    EXECUTABLE pose_setter_server
)

install(TARGETS
  #navigate_action navigate_action_server
  pose_setter pose_setter_server
  behavior_tree

  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
