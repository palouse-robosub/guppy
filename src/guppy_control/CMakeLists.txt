cmake_minimum_required(VERSION 3.8)
project(guppy_control)

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
find_package(proxsuite REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(guppy_msgs REQUIRED)
find_package(guppy_util REQUIRED)

# targets

add_executable(control_node
  src/control_node.cpp
  src/chassis_controller.cpp
  src/t200_interface.cpp
)

target_include_directories(control_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_node
  std_srvs
  control_toolbox
  geometry_msgs
  nav_msgs
  guppy_msgs
  Eigen3
  proxsuite
)

target_link_libraries(control_node
  guppy_util::guppy_util
)

set_source_files_properties(src/chassis_controller.cpp
  PROPERTIES
    COMPILE_OPTIONS "-O3;-DNDEBUG;-DPROXSUITE_VECTORIZE"
)

# installation

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
