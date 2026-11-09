cmake_minimum_required(VERSION 3.8)
project(amir_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/arm_visual_publisher.py
    scripts/capture_yolo_dataset.py
    scripts/gripper_command_server.py
    scripts/label_bottle_images.py
    scripts/octomap_cbf_sphere_generator.py
    scripts/odom_tf_relay.py
    scripts/pointcloud_mux.py
    scripts/semantic_pointcloud_filter.py
    scripts/sim_launch.sh
    scripts/split_yolo_dataset.py
    scripts/target_pose_publisher.py
    scripts/train_bottle_yolo.py
    scripts/yolo26_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
