cmake_minimum_required(VERSION 3.10)

# Package / project definition
project(p2_remote_connection)

# C++ standard + compiler warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # keep it standard C++ (no GNU extensions)

# Enable common warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find build system + ROS 2 dependencies (ament / message packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Unitree robot-specific dependencies (only needed for the REAL robot bridge)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

# Executable: web_teleop_bridge (REAL robot bridge)
# - Subscribes to /web_teleop and /web_action and forwards to Unitree Sport API
add_executable(web_teleop_bridge
  src/web_teleop_bridge.cpp
  src/common/ros2_sport_client.cpp
)

# Header search paths for this target
target_include_directories(web_teleop_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/p2_remote_connection
)

# Link this target against the ROS 2 + Unitree dependencies it uses
ament_target_dependencies(web_teleop_bridge
  rclcpp
  geometry_msgs
  std_msgs
  unitree_api
  unitree_go
)

# Executable: web_teleop_bridge_sim (SIM / non-robot version)
# - Same idea as the robot bridge but without Unitree dependencies
add_executable(web_teleop_bridge_sim
  src/web_teleop_bridge_sim.cpp
)

ament_target_dependencies(web_teleop_bridge_sim
  rclcpp
  geometry_msgs
  std_msgs
)

# Executable: move_forward_meters_node
# - Subscribes to /move_forward_meters (Float32)
# - Uses /odom (nav_msgs/Odometry) to measure distance travelled
# - Publishes /web_teleop (Twist) until distance reached, then stops
add_executable(move_forward_meters_node
  src/move_forward_meters_node.cpp
)

ament_target_dependencies(move_forward_meters_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# Install rules
# - install headers so other packages can include them
# - install binaries into the standard ROS 2 lib/<pkg> directory
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  web_teleop_bridge
  web_teleop_bridge_sim
  move_forward_meters_node
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize ament package (exports / package metadata)
ament_package()
