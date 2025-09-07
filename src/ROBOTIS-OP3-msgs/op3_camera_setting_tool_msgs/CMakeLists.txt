cmake_minimum_required(VERSION 3.5)
project(op3_camera_setting_tool_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraParams.msg"
  "msg/V4lParameter.msg"
  "msg/V4lParameters.msg"
  "srv/GetParameters.srv"
  "srv/SetParameters.srv"
  DEPENDENCIES std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()