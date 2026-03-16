cmake_minimum_required(VERSION 3.10)

# ------------------------------------------------------------
# Package definition
# ------------------------------------------------------------
project(go2_remote_connection)

# ------------------------------------------------------------
# C++ standard and warnings
# ------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# Core build system and ROS 2 dependencies
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(map_msgs REQUIRED)

# TF
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# ------------------------------------------------------------
# Unitree robot dependencies (real robot only)
# ------------------------------------------------------------
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

# ------------------------------------------------------------
# Executable: web_bridge
# ------------------------------------------------------------
add_executable(web_bridge
  src/web_bridge.cpp
  src/common/ros2_sport_client.cpp
)

target_include_directories(web_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/go2_remote_connection
)

ament_target_dependencies(web_bridge
  rclcpp
  geometry_msgs
  std_msgs
  unitree_api
  unitree_go
)

# ------------------------------------------------------------
# Executable: web_teleop_bridge_sim (no Unitree deps)
# ------------------------------------------------------------
add_executable(web_teleop_bridge_sim
  src/web_teleop_bridge_sim.cpp
)

ament_target_dependencies(web_teleop_bridge_sim
  rclcpp
  geometry_msgs
  std_msgs
)

# ------------------------------------------------------------
# Executable: move_forward_meters_node
# ------------------------------------------------------------
add_executable(move_forward_meters_node
  src/move_forward_meters_node.cpp
)

ament_target_dependencies(move_forward_meters_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# ------------------------------------------------------------
# Executable: flatten_l1_data (2D occupancy map)
# ------------------------------------------------------------
add_executable(flatten_l1_data
  src/flatten_L1_data.cpp
)

ament_target_dependencies(flatten_l1_data
  rclcpp
  sensor_msgs
  nav_msgs
  map_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)

# ------------------------------------------------------------
# Executable: L1_pcd_web_packer (3D point cloud streamer)
# ------------------------------------------------------------
add_executable(L1_pcd_web_packer
  src/L1_pcd_web_packer.cpp
)

ament_target_dependencies(L1_pcd_web_packer
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# ------------------------------------------------------------
# Install rules
# ------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  web_bridge
  web_teleop_bridge_sim
  move_forward_meters_node
  flatten_l1_data
  L1_pcd_web_packer
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Finalize package
# ------------------------------------------------------------
ament_package()
