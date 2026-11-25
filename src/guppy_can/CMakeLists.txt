cmake_minimum_required(VERSION 3.23)
project(guppy_can)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_xml REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# components

add_library(can_transceiver_component SHARED
  src/can_transceiver.cpp
)
target_compile_features(can_transceiver_component PUBLIC cxx_std_17)
target_compile_options(can_transceiver_component PRIVATE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)
target_sources(can_transceiver_component
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/guppy_can/can_transceiver.hpp
)
target_link_libraries(can_transceiver_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    guppy_msgs::guppy_msgs
    guppy_util::guppy_util
    guppy_testing::guppy_testing
    ament_index_cpp::ament_index_cpp
    can_xml::can_xml
)
rclcpp_components_register_node(can_transceiver_component
  PLUGIN "guppy_can::CANTransceiver"
  EXECUTABLE can_transceiver_node
)

# installation

install(TARGETS can_transceiver_component
  EXPORT export_${PROJECT_NAME}
  FILE_SET HEADERS DESTINATION include/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(guppy_testing REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}
    test/can_transceiver_test.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
      can_transceiver_component
      rclcpp::rclcpp
      guppy_msgs::guppy_msgs
      guppy_testing::guppy_testing
      can_xml::can_xml
  )
endif()

# package

ament_package()
