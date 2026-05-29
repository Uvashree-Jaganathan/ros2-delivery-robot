cmake_minimum_required(VERSION 3.8)
project(delivery_bot_mission_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_manager src/mission_manager.cpp)

ament_target_dependencies(mission_manager
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
