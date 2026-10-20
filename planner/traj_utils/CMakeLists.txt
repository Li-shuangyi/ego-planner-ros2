cmake_minimum_required(VERSION 3.22)
project(traj_utils LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  "msg/MotionCommand.msg"
  "msg/MINCOTraj.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)
add_library(traj_utils_lib SHARED
  src/planning_visualization.cpp)
target_include_directories(traj_utils_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(traj_utils_lib rclcpp std_msgs geometry_msgs visualization_msgs)
target_link_libraries(traj_utils_lib Eigen3::Eigen "${typesupport}")
install(TARGETS traj_utils_lib EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs visualization_msgs Eigen3)
ament_package()
