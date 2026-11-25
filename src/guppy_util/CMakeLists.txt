cmake_minimum_required(VERSION 3.23)
project(guppy_util LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# targets

add_library(${PROJECT_NAME} SHARED
  src/socket.cpp
  src/can.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)
target_sources(${PROJECT_NAME}
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/guppy_util/socket.hpp
      include/guppy_util/socket_interface.hpp
      include/guppy_util/quality.hpp
      include/guppy_util/can.hpp
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
)

# python stuff

#ament_python_install_package(${PROJECT_NAME})

#install(
#  PROGRAMS
#    ${PROJECT_NAME}/example.py
#  DESTINATION lib/${PROJECT_NAME}
#)

# installation

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  FILE_SET HEADERS DESTINATION include/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# exports

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
)

# testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# for a header-only package, swap the first block for add_library(${PROJECT_NAME} INTERFACE), make the target_* keywords INTERFACE, and remove HAS_LIBRARY_TARGET.
