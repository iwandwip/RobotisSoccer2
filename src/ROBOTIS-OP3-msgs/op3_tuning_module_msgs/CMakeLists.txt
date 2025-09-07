cmake_minimum_required(VERSION 3.5)
project(op3_tuning_module_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointOffsetData.msg"
  "msg/JointOffsetPositionData.msg"
  "msg/JointTorqueOnOff.msg"
  "msg/JointTorqueOnOffArray.msg"
  "srv/GetPresentJointOffsetData.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
