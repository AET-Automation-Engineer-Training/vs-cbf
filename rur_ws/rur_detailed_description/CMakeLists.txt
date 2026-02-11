cmake_minimum_required(VERSION 3.5)
project(rur_detailed_description)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED)

# Install launch, URDF, meshes, gazebo
install(
  DIRECTORY
    launch
    urdf
    meshes
    gazebo  # <-- Thêm dòng này để cài thư mục gazebo
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
