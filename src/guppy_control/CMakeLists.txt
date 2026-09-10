cmake_minimum_required(VERSION 3.8)
project(guppy_control)

# Default to C++17 for ROS2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  endif()

  # Enable warnings
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(proxsuite REQUIRED)
    find_package(control_toolbox REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(guppy_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(eigen3_cmake_module REQUIRED)
    find_package(Eigen3)

    include_directories(include)

    add_executable(control_node src/control_node.cpp src/chassis_controller.cpp src/t200_interface.cpp)
    ament_target_dependencies(control_node std_srvs control_toolbox geometry_msgs nav_msgs guppy_msgs Eigen3 proxsuite)
    set_source_files_properties(src/chassis_controller.cpp PROPERTIES
      COMPILE_OPTIONS "-O3;-DNDEBUG;-DPROXSUITE_VECTORIZE"
    )
    install(DIRECTORY
      launch

      DESTINATION share/${PROJECT_NAME}/
    )
    install(TARGETS
      control_node

      DESTINATION lib/${PROJECT_NAME}
    )
    install(DIRECTORY
      config

      DESTINATION share/${PROJECT_NAME}/
    )
    ament_package()
