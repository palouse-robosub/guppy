cmake_minimum_required(VERSION 3.8)
project(guppy_util)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can.cpp
)

target_compile_features(${PROJECT_NAME}
  PUBLIC
    cxx_std_17
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}_targets
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(
  ${PROJECT_NAME}_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
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
