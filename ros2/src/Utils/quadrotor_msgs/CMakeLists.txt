cmake_minimum_required(VERSION 3.22)
project(quadrotor_msgs LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StatusData.msg"
  "msg/AuxCommand.msg"
  "msg/GoalSet.msg"
  "msg/PPROutputData.msg"
  "msg/LQRTrajectory.msg"
  "msg/OutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/TRPYCommand.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/Odometry.msg"
  "msg/PositionCommand.msg"
  "msg/Gains.msg"
  "msg/Corrections.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
