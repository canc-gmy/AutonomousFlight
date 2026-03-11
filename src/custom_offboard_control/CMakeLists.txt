cmake_minimum_required(VERSION 3.8)
project(custom_offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executables
add_executable(offboard_control_node src/offboard_control.cpp)
ament_target_dependencies(offboard_control_node 
  rclcpp 
  px4_msgs 
  geometry_msgs 
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(offboard_control_with_feedback_node src/offboard_control_with_feedback.cpp)
ament_target_dependencies(offboard_control_with_feedback_node 
  rclcpp 
  px4_msgs 
  geometry_msgs 
  sensor_msgs
)

add_executable(offboard_control_with_geofence_node src/offboard_control_with_geofence.cpp)
ament_target_dependencies(offboard_control_with_geofence_node 
  rclcpp 
  std_msgs
  px4_msgs 
  geometry_msgs 
  sensor_msgs
)

add_executable(offboard_control_with_car_node src/offboard_control_with_car.cpp)
ament_target_dependencies(offboard_control_with_car_node 
  rclcpp 
  std_msgs
  px4_msgs 
  geometry_msgs 
  sensor_msgs
  nav_msgs
  visualization_msgs
  gz-transport13
  gz-msgs10
)

add_executable(offboard_control_sequenced_node src/offboard_control_sequenced.cpp)
ament_target_dependencies(offboard_control_sequenced_node 
  rclcpp 
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  px4_msgs 
  geometry_msgs 
  sensor_msgs
  nav_msgs
  visualization_msgs
  gz-transport13
  gz-msgs10
)

# Install executables
install(TARGETS
  offboard_control_node
  offboard_control_with_feedback_node
  offboard_control_with_geofence_node
  offboard_control_with_car_node
  offboard_control_sequenced_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
