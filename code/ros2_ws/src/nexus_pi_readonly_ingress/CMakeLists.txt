cmake_minimum_required(VERSION 3.8)
project(nexus_pi_readonly_ingress)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS
  src/${PROJECT_NAME}/telemetry_ingress_node.py
  src/${PROJECT_NAME}/fcu_clock_ingress_node.py
  src/${PROJECT_NAME}/range_smoothing_node.py
  src/${PROJECT_NAME}/camera_file_node.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ingress test/test_ingress.py)
  ament_add_pytest_test(test_range_smoothing test/test_range_smoothing.py)
  ament_add_pytest_test(test_fcu_clock_ingress test/test_fcu_clock_ingress.py)
  ament_add_pytest_test(test_camera_clock test/test_camera_clock.py)
  ament_add_pytest_test(test_camera_intrinsics test/test_camera_intrinsics.py)
  ament_add_pytest_test(test_camera_latest_frame test/test_camera_latest_frame.py)
endif()

ament_package()
