cmake_minimum_required(VERSION 3.8)
project(simp_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReferencePath.msg"
  "msg/ExecutedCommand.msg"
  "msg/DriveModeState.msg"
  "msg/TrackingTrajectoryPoint.msg"
  "msg/TrackingTrajectory.msg"
  "msg/TrackingDiagnosticsPoint.msg"
  "msg/TrackingDiagnostics.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
