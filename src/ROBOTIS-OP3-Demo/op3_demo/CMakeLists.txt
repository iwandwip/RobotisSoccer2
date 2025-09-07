cmake_minimum_required(VERSION 3.5)
project(op3_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_ball_detector REQUIRED)
find_package(op3_ball_detector_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${op3_walking_module_msgs_INCLUDE_DIRS}
  ${op3_action_module_msgs_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${op3_ball_detector_INCLUDE_DIRS}
  ${op3_ball_detector_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(op_demo_node 
  src/demo_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
)

ament_target_dependencies(op_demo_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_action_module_msgs
  robotis_math
  op3_ball_detector
  op3_ball_detector_msgs
  Boost
  Eigen3
  YAML_CPP
)

add_executable(self_test_node 
  src/test_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
  src/test/button_test.cpp
  src/test/mic_test.cpp
)

ament_target_dependencies(self_test_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_action_module_msgs
  robotis_math
  op3_ball_detector
  op3_ball_detector_msgs
  Boost
  Eigen3
  YAML_CPP
)

install(TARGETS op_demo_node self_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY data launch list 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()