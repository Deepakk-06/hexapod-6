cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoPosition.msg"
  "msg/ServosPosition.msg"
)

ament_package()
