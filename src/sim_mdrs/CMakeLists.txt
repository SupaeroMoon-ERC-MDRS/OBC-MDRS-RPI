cmake_minimum_required(VERSION 3.5)
project(sim_mdrs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(osr_controller src/osr_controller.cpp)
ament_target_dependencies(osr_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
  trajectory_msgs
  control_msgs
)

install(TARGETS osr_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Export the package
ament_package()
