cmake_minimum_required(VERSION 3.5)
project(op3_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY doc launch meshes rviz urdf
	DESTINATION share/${PROJECT_NAME}
)
