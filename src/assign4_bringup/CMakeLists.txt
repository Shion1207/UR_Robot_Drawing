cmake_minimum_required(VERSION 3.8)
project(assign4_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_explore.py
    scripts/wasd_teleop.py
    scripts/waypoint_navigator.py
    scripts/yolo_detector.py
    scripts/gesture_recognition_node.py
    scripts/robot_behavior_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/gesture_logic.py DESTINATION lib/${PROJECT_NAME})

ament_package()

