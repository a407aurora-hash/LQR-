cmake_minimum_required(VERSION 3.8)
project(mujoco_control)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mujoco_control_node src/mujoco_control_node.cpp)

ament_target_dependencies(mujoco_control_node rclcpp std_msgs)

install(TARGETS
  mujoco_control_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
