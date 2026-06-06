cmake_minimum_required(VERSION 3.8)
project(so101_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/PixelPoint.msg"
  "msg/TaskState.msg"
  "srv/DetectObject.srv"
  "srv/InitializeTracking.srv"
  "srv/PixelToRobotXY.srv"
  "srv/SetDropZone.srv"
  "srv/StartTask.srv"
  DEPENDENCIES std_msgs
)

ament_package()
