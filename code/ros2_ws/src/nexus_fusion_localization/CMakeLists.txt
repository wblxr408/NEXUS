cmake_minimum_required(VERSION 3.8)
project(nexus_fusion_localization)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/fusion_node.py DESTINATION lib/${PROJECT_NAME} RENAME fusion_localization_node)
install(PROGRAMS src/${PROJECT_NAME}/train_reliability.py DESTINATION lib/${PROJECT_NAME} RENAME train_reliability)
install(PROGRAMS src/${PROJECT_NAME}/platform_node.py DESTINATION lib/${PROJECT_NAME} RENAME platform_localization_node)
install(PROGRAMS src/${PROJECT_NAME}/target_kinematic_node.py DESTINATION lib/${PROJECT_NAME} RENAME target_kinematic_fusion_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_add_pytest_test(test_fusion test/test_fusion.py)
ament_add_pytest_test(test_robust_filter test/test_robust_filter.py)
ament_add_pytest_test(test_reliability test/test_reliability.py)
ament_add_pytest_test(test_fusion_node test/test_fusion_node.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=213)
ament_add_pytest_test(test_platform_window test/test_platform_window.py
  ENV OPENBLAS_NUM_THREADS=1 TIMEOUT 120)
ament_add_pytest_test(test_platform_jacobians test/test_platform_jacobians.py
  ENV OPENBLAS_NUM_THREADS=1)
ament_add_pytest_test(test_platform_node test/test_platform_node.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=214 OPENBLAS_NUM_THREADS=1)
ament_add_pytest_test(test_kinematic_filter test/test_kinematic_filter.py
  ENV OPENBLAS_NUM_THREADS=1)
ament_add_pytest_test(test_target_kinematic_node test/test_target_kinematic_node.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=228 OPENBLAS_NUM_THREADS=1)
ament_package()
