cmake_minimum_required(VERSION 3.8)
project(guppy_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)

# Spinnaker paths
set(SPINNAKER_INCLUDE_DIR /opt/spinnaker/include)
set(SPINNAKER_LIBRARY /opt/spinnaker/lib/libSpinnaker.so)

# Create imported library target
add_library(Spinnaker::Spinnaker SHARED IMPORTED)

set_target_properties(Spinnaker::Spinnaker PROPERTIES
        IMPORTED_LOCATION ${SPINNAKER_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${SPINNAKER_INCLUDE_DIR}
)

add_executable(camera_pub src/camera_pub.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs image_transport)

target_link_libraries(camera_pub
        Spinnaker::Spinnaker
)

add_executable(webcam_pub src/webcam_pub.cpp)
add_executable(pnp src/pnp.cpp)
add_executable(slam src/slam.cpp)
add_executable(aprilrepub src/april.cpp)

ament_target_dependencies(webcam_pub rclcpp sensor_msgs cv_bridge OpenCV image_transport)
ament_target_dependencies(aprilrepub rclcpp sensor_msgs cv_bridge OpenCV image_transport)
ament_target_dependencies(pnp rclcpp guppy_msgs sensor_msgs cv_bridge OpenCV Eigen3)
ament_target_dependencies(slam rclcpp nav_msgs guppy_msgs sensor_msgs Eigen3 tf2_geometry_msgs tf2 tf2_ros)

install(TARGETS
  webcam_pub
  aprilrepub
  pnp
  slam
  camera_pub
  
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/apriltag_detector.py
  ${PROJECT_NAME}/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
