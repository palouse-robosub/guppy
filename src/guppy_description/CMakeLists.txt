cmake_minimum_required(VERSION 3.8)
project(guppy_description)

# dependencies

find_package(ament_cmake REQUIRED)

# installation

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
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
