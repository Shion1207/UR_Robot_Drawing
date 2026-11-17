cmake_minimum_required(VERSION 3.8)
project(assign8_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/emotion_recognition_node.py scripts/emotion_motion_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/emotion_logic.py scripts/camera_geometry.py DESTINATION lib/${PROJECT_NAME})
ament_package()
