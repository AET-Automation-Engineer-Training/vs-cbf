cmake_minimum_required(VERSION 3.5)
project(data_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  
find_package(sensor_msgs REQUIRED)         
find_package(visualization_msgs REQUIRED)  
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  
find_package(tf2 REQUIRED)   
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)   

# Node 1: obstacle_ellipses_node
add_executable(obstacle_ellipses_node src/obstacle_ellipses_node.cpp)
ament_target_dependencies(obstacle_ellipses_node
  rclcpp
  std_msgs
  pedsim_msgs
  geometry_msgs
  tf2
)

# Node 2: tracked_person_publisher
add_executable(tracked_person_publisher src/tracked_person_publisher.cpp)
ament_target_dependencies(tracked_person_publisher
  rclcpp
  pedsim_msgs
  geometry_msgs
  builtin_interfaces
)

# Node 3: tracked_group_publisher
add_executable(tracked_group_publisher src/tracked_group_publisher.cpp)
ament_target_dependencies(tracked_group_publisher
  rclcpp
  pedsim_msgs
  geometry_msgs
  builtin_interfaces
)

# Node 4: line_obstacles_to_point_cloud
add_executable(line_obstacles_to_point_cloud src/line_obstacles_to_point_cloud.cpp)
ament_target_dependencies(line_obstacles_to_point_cloud
  rclcpp
  pedsim_msgs
  sensor_msgs
)

# Node 5: all data needed from simulation
add_executable(navigation_metrics_logger_node src/navigation_metrics_logger_node.cpp)
ament_target_dependencies(navigation_metrics_logger_node
  rclcpp
  nav_msgs
  pedsim_msgs
  rclcpp_action
  nav2_msgs
)

# Install compiled nodes
install(TARGETS
  obstacle_ellipses_node
  tracked_person_publisher
  tracked_group_publisher
  line_obstacles_to_point_cloud
  navigation_metrics_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
