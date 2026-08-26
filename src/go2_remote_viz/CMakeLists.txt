cmake_minimum_required(VERSION 3.10)

# ------------------------------------------------------------
# Package definition
#
# The visualisation / data half of the former go2_remote_connection package.
# Note what is absent: unitree_api. Nothing here talks to the robot, so a
# recorded session can be replayed and exported on any machine with ROS 2.
# ------------------------------------------------------------
project(go2_remote_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# ------------------------------------------------------------
# Executable: flatten_l1_data (L1 lidar cloud -> 2D occupancy map)
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
# Install rules
# ------------------------------------------------------------
install(TARGETS
  flatten_l1_data
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/mock_map2d_pub.py
  recording/bag_to_dataset.py
  recording/lowstate_to_jointstate.py
  recording/sportmodestate_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch + RViz config for session replay.
install(DIRECTORY
  recording
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
