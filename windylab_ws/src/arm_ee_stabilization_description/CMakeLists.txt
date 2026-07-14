cmake_minimum_required(VERSION 3.8)
project(arm_ee_stabilization_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
