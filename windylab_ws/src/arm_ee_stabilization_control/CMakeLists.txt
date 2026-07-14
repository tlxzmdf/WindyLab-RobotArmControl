cmake_minimum_required(VERSION 3.8)
project(arm_ee_stabilization_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(ee_stabilization
  src/ee_stabilization_node.cpp
  src/EeStabilizationNode.cpp
  src/PinocchioDynamicsModel.cpp
  src/PinocchioIkSolver.cpp
  src/BaseDisturbanceGenerator.cpp
  src/StabilizationModeD.cpp
)
target_include_directories(ee_stabilization PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  ee_stabilization
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(ee_stabilization pinocchio::pinocchio tf2::tf2)

install(TARGETS ee_stabilization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
