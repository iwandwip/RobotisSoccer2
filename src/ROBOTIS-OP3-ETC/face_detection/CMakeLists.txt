cmake_minimum_required(VERSION 3.5)
project(face_detection)

#set(CMAKE_CXX_FLAGS "-std=gnu++0x ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -o0")

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(face_tracking src/face_tracking.cpp)

ament_target_dependencies(face_tracking
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp 
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(face_tracking
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

install(TARGETS face_tracking
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()