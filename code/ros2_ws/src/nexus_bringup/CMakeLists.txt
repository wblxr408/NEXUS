cmake_minimum_required(VERSION 3.8)
project(nexus_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/preflight_node.py DESTINATION lib/${PROJECT_NAME} RENAME preflight_node)
install(PROGRAMS src/${PROJECT_NAME}/uav_route_node.py DESTINATION lib/${PROJECT_NAME} RENAME uav_route_node)
install(PROGRAMS src/${PROJECT_NAME}/telemetry_udp_node.py DESTINATION lib/${PROJECT_NAME} RENAME telemetry_udp_node)
install(PROGRAMS src/${PROJECT_NAME}/approximate_initial_pose_node.py DESTINATION lib/${PROJECT_NAME} RENAME approximate_initial_pose_node)
install(PROGRAMS src/${PROJECT_NAME}/nlos_range_replay_node.py DESTINATION lib/${PROJECT_NAME} RENAME nlos_range_replay_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../../simulation/
  DESTINATION share/${PROJECT_NAME}/simulation
  FILES_MATCHING PATTERN "*.world" PATTERN "*.yaml" PATTERN "__pycache__" EXCLUDE)
ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
ament_add_pytest_test(test_gazebo_contract test/test_gazebo_contract.py)
add_launch_test(test/test_realtime_pipeline_launch.py TIMEOUT 30)
ament_add_pytest_test(test_dual_launch test/test_dual_launch.py)
ament_add_pytest_test(test_approximate_initial_pose test/test_approximate_initial_pose.py TIMEOUT 40
  ENV "ROS_DOMAIN_ID=231" "OPENBLAS_NUM_THREADS=1" "FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA")
ament_add_pytest_test(test_dual_recording test/test_dual_recording.py TIMEOUT 60
  ENV "ROS_LOCALHOST_ONLY=0" "ROS_DOMAIN_ID=230" "FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA" "OPENBLAS_NUM_THREADS=1")
ament_package()
