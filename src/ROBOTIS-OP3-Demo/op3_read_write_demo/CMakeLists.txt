cmake_minimum_required(VERSION 3.5)
project(op3_read_write_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
)

add_executable(read_write src/read_write.cpp)

ament_target_dependencies(read_write
  rclcpp
  sensor_msgs
  std_msgs
  robotis_controller_msgs
)

install(TARGETS read_write
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
