cmake_minimum_required(VERSION 3.8)
project(assign4_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_explore.py
    scripts/wasd_teleop.py
    scripts/waypoint_navigator.py
    scripts/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

