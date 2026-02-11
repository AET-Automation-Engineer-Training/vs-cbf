cmake_minimum_required(VERSION 3.8)
project(nav2_mpc_cbf_controller)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate our .srv in THIS package
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MpcCbfOptimize.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Include dirs
include_directories(include)

# Build the Nav2 controller plugin (use a unique target name)
add_library(${PROJECT_NAME}_plugin SHARED
  src/mpc_cbf_controller.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d
  geometry_msgs nav_msgs std_msgs
)

# Link to our own typesupport target (modern Humble pattern)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin ${typesupport_target})

# Export plugin description
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

# Install C++ bits
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Install the Python optimizer script as an executable
install(PROGRAMS scripts/mpc_cbf_optimizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export includes for downstream users
ament_export_include_directories(include)

# MUST be last
ament_package()
