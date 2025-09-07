cmake_minimum_required(VERSION 3.5)
project(op3_online_walking_module_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/JointPose.msg"
  "msg/KinematicsPose.msg"
  "msg/FootStepCommand.msg"
  "msg/FootStepArray.msg"
  "msg/PreviewRequest.msg"
  "msg/PreviewResponse.msg"
  "msg/WalkingParam.msg"
  "msg/Step2D.msg"
  "msg/Step2DArray.msg"
)

set(srv_files
  "srv/GetJointPose.srv"
  "srv/GetKinematicsPose.srv"
  "srv/GetPreviewMatrix.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
