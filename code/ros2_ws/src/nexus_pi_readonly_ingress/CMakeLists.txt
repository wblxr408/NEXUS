cmake_minimum_required(VERSION 3.8)
project(nexus_pi_readonly_ingress)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS
  src/${PROJECT_NAME}/telemetry_ingress_node.py
  src/${PROJECT_NAME}/camera_file_node.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ingress test/test_ingress.py)
endif()

ament_package()
