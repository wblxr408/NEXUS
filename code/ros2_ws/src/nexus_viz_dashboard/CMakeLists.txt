cmake_minimum_required(VERSION 3.8)
project(nexus_viz_dashboard)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/dashboard_node.py DESTINATION lib/${PROJECT_NAME} RENAME dashboard_node)
install(DIRECTORY config launch rviz web DESTINATION share/${PROJECT_NAME})
ament_add_pytest_test(test_dashboard_contract test/test_dashboard_contract.py)
install(PROGRAMS src/${PROJECT_NAME}/localization_dashboard_node.py DESTINATION lib/${PROJECT_NAME} RENAME localization_dashboard_node)
ament_add_pytest_test(test_localization_dashboard test/test_localization_dashboard.py
  ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=229")
find_program(NEXUS_NODE_EXECUTABLE node REQUIRED)
add_test(NAME test_localization_web COMMAND ${NEXUS_NODE_EXECUTABLE} --experimental-default-type=module --test
  ${CMAKE_CURRENT_SOURCE_DIR}/web/test/localization.test.mjs
  ${CMAKE_CURRENT_SOURCE_DIR}/web/test/registration.test.mjs)
ament_package()
