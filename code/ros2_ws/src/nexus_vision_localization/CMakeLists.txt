cmake_minimum_required(VERSION 3.8)
project(nexus_vision_localization)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS src/${PROJECT_NAME}/vision_node.py DESTINATION lib/${PROJECT_NAME} RENAME vision_localization_node)
install(PROGRAMS src/${PROJECT_NAME}/fixed_reference_node.py DESTINATION lib/${PROJECT_NAME} RENAME fixed_reference_node)
install(PROGRAMS src/${PROJECT_NAME}/detection_node.py DESTINATION lib/${PROJECT_NAME} RENAME object_detection_node)
install(PROGRAMS src/${PROJECT_NAME}/superpoint_node.py DESTINATION lib/${PROJECT_NAME} RENAME superpoint_motion_node)
install(PROGRAMS src/${PROJECT_NAME}/target_metric_node.py DESTINATION lib/${PROJECT_NAME} RENAME target_metric_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_add_pytest_test(test_detector test/test_detector.py)
ament_add_pytest_test(test_reference_geometry test/test_reference_geometry.py)
ament_add_pytest_test(test_superpoint_frontend test/test_superpoint_frontend.py)
ament_add_pytest_test(test_visual_motion test/test_visual_motion.py)
ament_add_pytest_test(test_object_detector test/test_object_detector.py)
ament_add_pytest_test(test_static_tracker test/test_static_tracker.py)
ament_add_pytest_test(test_target_identity test/test_target_identity.py)
ament_add_pytest_test(test_target_frontend test/test_target_frontend.py)
ament_add_pytest_test(test_target_geometry test/test_target_geometry.py)
ament_add_pytest_test(test_target_multiview test/test_target_multiview.py)
ament_add_pytest_test(test_target_metric_node test/test_target_metric_node.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=218 OPENBLAS_NUM_THREADS=1 TIMEOUT 90)
ament_add_pytest_test(test_target_pipeline_nodes test/test_target_pipeline_nodes.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 OPENBLAS_NUM_THREADS=1 TIMEOUT 90)
ament_add_pytest_test(test_visual_pipeline_nodes test/test_visual_pipeline_nodes.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216 OPENBLAS_NUM_THREADS=1 TIMEOUT 90)
ament_add_pytest_test(test_fixed_reference_node test/test_fixed_reference_node.py
  ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=215 OPENBLAS_NUM_THREADS=1)
ament_package()
