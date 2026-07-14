cmake_minimum_required(VERSION 3.8)
project(arm_teleop_wbc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(teleop_wbc
  src/teleop_wbc_node.cpp
  src/TeleopWbcNode.cpp
  src/PinocchioArmModel.cpp
  src/VelocityWbcController.cpp
  src/BoxQpSolver.cpp
  src/BaseDisturbanceGenerator.cpp
)
target_include_directories(teleop_wbc PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  teleop_wbc
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)
target_link_libraries(teleop_wbc pinocchio::pinocchio)

install(TARGETS teleop_wbc DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
