cmake_minimum_required(VERSION 3.8)
project(nexus_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/preflight_node.py DESTINATION lib/${PROJECT_NAME} RENAME preflight_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
add_launch_test(test/test_realtime_pipeline_launch.py TIMEOUT 30)
ament_package()
