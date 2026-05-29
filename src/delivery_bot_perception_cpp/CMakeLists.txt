cmake_minimum_required(VERSION 3.8)
project(delivery_bot_perception_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(yolo_onnx_node src/yolo_onnx_node.cpp)
ament_target_dependencies(yolo_onnx_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  image_transport
)
target_link_libraries(yolo_onnx_node ${OpenCV_LIBS})

add_executable(safety_stop_node src/safety_stop_node.cpp)
ament_target_dependencies(safety_stop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  yolo_onnx_node
  safety_stop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
