cmake_minimum_required(VERSION 3.18)
project(robot_sam2_ros)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom message and action interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
  "msg/GraspCommand.msg"
  "action/MoveToTarget.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python package.
ament_python_install_package(${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
