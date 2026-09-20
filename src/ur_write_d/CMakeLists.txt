cmake_minimum_required(VERSION 3.8)
project(ur_write_d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ur_write_d_node src/ur_write_d_node.cpp)
target_include_directories(ur_write_d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur_write_d_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ur_write_d_node
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_ros"
  "visualization_msgs"
)

install(TARGETS
  ur_write_d_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
