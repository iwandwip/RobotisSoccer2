cmake_minimum_required(VERSION 3.5)
project(op3_offset_tuner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/JointOffsetData.msg"
  "msg/JointOffsetPositionData.msg"
  "msg/JointTorqueOnOff.msg"
  "msg/JointTorqueOnOffArray.msg"
)

set(srv_files
  "srv/GetPresentJointOffsetData.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
