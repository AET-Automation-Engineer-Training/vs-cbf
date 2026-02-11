################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rur_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/goal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
