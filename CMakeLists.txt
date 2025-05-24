cmake_minimum_required(VERSION 3.8)
project(jimbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(angles REQUIRED)
find_package(jimbot_msgs)



include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
)

add_executable(navigate_to_pose_server src/navigate_to_pose_server.cpp)

ament_target_dependencies(navigate_to_pose_server
  rclcpp
  rclcpp_action
  nav2_msgs
  jimbot_msgs
)

add_executable(local_map_node src/local_map_publisher.cpp)

ament_target_dependencies(local_map_node
  rclcpp
  std_msgs
  sensor_msgs
  PCL
  pcl_conversions
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
)
target_link_libraries(local_map_node yaml-cpp)



set(DEPENDENCIES
  "rclcpp_action"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "nav2_util"
  "angles"
  "jimbot_msgs"
)

# target_link_libraries(controller_server jim_control)


install(TARGETS
  navigate_to_pose_server
  local_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
)

ament_package()
