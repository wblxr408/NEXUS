cmake_minimum_required(VERSION 3.8)
project(nexus_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetObservation.msg"
  "msg/TargetKinematicState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_add_pytest_test(test_message_contract test/test_message_contract.py)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
