cmake_minimum_required(VERSION 3.16)
project(glaucus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BotTelemetry.msg"
  "msg/DriveCommand.msg"
  "msg/EndOfLaneEvidence.msg"
  "msg/SweepStatus.msg"
  "msg/SweepEvent.msg"
  "action/RunSweep.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
