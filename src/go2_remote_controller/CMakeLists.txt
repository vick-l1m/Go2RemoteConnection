cmake_minimum_required(VERSION 3.10)

# ------------------------------------------------------------
# Package definition
#
# The control half of the former go2_remote_connection package: everything that
# commands the robot. The recording/replay tooling now lives in go2_remote_viz,
# which deliberately has no Unitree SDK dependency so it builds on a laptop.
# ------------------------------------------------------------
project(go2_remote_controller)

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

# TF
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/include/go2_remote_controller
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
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------
# Install rules
# ------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include/
)

# YOLO model used by the dashboard's detection service.
install(DIRECTORY
  src/cv/model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(TARGETS
  web_bridge
  web_teleop_bridge_sim
  move_forward_meters_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/cv/front_camera_capture.py
  src/cv/front_camera_ros_bridge.py
  src/cmd_vel_file_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# The RL runtime and the FastAPI app are run from source by the launcher scripts
# (they need their sibling data files), so ship them as package data.
install(DIRECTORY
  rl_policy
  app
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------
# Finalize package
# ------------------------------------------------------------
ament_package()
