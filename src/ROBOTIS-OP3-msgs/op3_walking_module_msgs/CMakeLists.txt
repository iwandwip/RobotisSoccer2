cmake_minimum_required(VERSION 3.5)
project(op3_walking_module_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WalkingParam.msg"
  "srv/GetWalkingParam.srv"
  "srv/SetWalkingParam.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
