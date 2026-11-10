cmake_minimum_required(VERSION 3.8)
project(assign7_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch maps models rviz semantic worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/segment_map.py scripts/semantic_regions_node.py
  scripts/voice_navigation_node.py scripts/speech_recognition_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/semantic_map.py scripts/voice_commands.py DESTINATION lib/${PROJECT_NAME})
ament_package()
