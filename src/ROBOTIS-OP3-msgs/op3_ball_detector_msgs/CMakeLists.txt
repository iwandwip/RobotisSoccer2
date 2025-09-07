cmake_minimum_required(VERSION 3.5)
project(op3_ball_detector_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallDetectorParams.msg"
  "msg/CircleSetStamped.msg"
  "srv/GetParameters.srv"
  "srv/SetParameters.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()