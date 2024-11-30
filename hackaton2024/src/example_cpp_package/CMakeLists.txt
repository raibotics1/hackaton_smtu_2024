cmake_minimum_required(VERSION 3.5)
project(example_cpp_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(example_cpp_node src/example_cpp_node.cpp)
ament_target_dependencies(example_cpp_node rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

install(TARGETS
  example_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
