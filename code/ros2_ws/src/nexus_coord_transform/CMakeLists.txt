cmake_minimum_required(VERSION 3.8)
project(nexus_coord_transform)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/coord_transform_node.py DESTINATION lib/${PROJECT_NAME} RENAME coord_transform_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_add_pytest_test(test_geometry test/test_geometry.py)
ament_package()
