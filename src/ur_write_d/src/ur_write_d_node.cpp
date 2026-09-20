#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Pose makePose(
  const geometry_msgs::msg::Quaternion & orientation, double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation = orientation;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return pose;
}

std::vector<geometry_msgs::msg::Pose> makeCleanD(
  const geometry_msgs::msg::Quaternion & orientation, double plane_x, double center_y,
  double bottom_z, double height, double width)
{
  std::vector<geometry_msgs::msg::Pose> points;
  const double left = center_y - width / 2.0;
  const double middle_z = bottom_z + height / 2.0;
  const double radius = height / 2.0;

  // 1. Vertical straight stem (bottom to top)
  const int line_steps = 25;
  for (int i = 0; i <= line_steps; ++i) {
    const double t = static_cast<double>(i) / line_steps;
    const double y = left;
    const double z = bottom_z + t * height;
    points.push_back(makePose(orientation, plane_x, y, z));
  }

  // 2. Semicircle bowl of D (top back to bottom)
  const int arc_steps = 35;
  for (int i = 1; i <= arc_steps; ++i) {
    const double t = static_cast<double>(i) / arc_steps;
    const double angle = (kPi / 2.0) - t * kPi;
    const double y = left + width * std::cos(angle);
    const double z = middle_z + radius * std::sin(angle);
    points.push_back(makePose(orientation, plane_x, y, z));
  }
  return points;
}

double pointDistance(
  const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = rclcpp::Node::make_shared("ur_write_d_node", options);

  // Robot-description parameters are auto-declared for MoveIt, so launch
  // overrides may already exist when these values are read.
  const auto parameterOr = [&node](const std::string & name, double fallback) {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<double>(name, fallback);
      }
      return node->get_parameter(name).as_double();
    };
  const double plane_x = parameterOr("plane_x", 0.30);
  const double center_y = parameterOr("letter_center_y", 0.05);
  const double bottom_z = parameterOr("letter_bottom_z", 0.20);
  const double height = parameterOr("letter_height", 0.15);
  const double width = parameterOr("letter_width", 0.15);
  const double lift = parameterOr("lift_distance", 0.05);
  const double line_width = parameterOr("line_width", 0.008);

  if (height <= 0.0 || width <= 0.0 || lift <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Letter dimensions and lift_distance must be positive.");
    rclcpp::shutdown();
    return 1;
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  const auto marker_pub =
    node->create_publisher<visualization_msgs::msg::Marker>("write_path_marker", qos);
  auto marker = std::make_shared<visualization_msgs::msg::Marker>();
  marker->ns = "duong_net_chu_d";
  marker->id = 0;
  marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = std::max(0.001, line_width);
  marker->color.r = 0.05F;
  marker->color.g = 0.90F;
  marker->color.b = 0.25F;
  marker->color.a = 1.0F;
  marker->frame_locked = true;
  marker->lifetime = rclcpp::Duration(0, 0); // Permanent marker

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  (void)tf_listener;
  std::atomic_bool pen_down{false};
  std::string planning_frame;
  std::string tool_link;

  // Record the actual tool position, not merely the requested waypoint.
  const auto trail_timer = node->create_wall_timer(
    25ms, [node, marker_pub, marker, tf_buffer, &pen_down, &planning_frame, &tool_link]() {
      if (!pen_down.load() || planning_frame.empty() || tool_link.empty()) {
        return;
      }
      try {
        const auto tf =
          tf_buffer->lookupTransform(planning_frame, tool_link, tf2::TimePointZero);
        geometry_msgs::msg::Point p;
        p.x = tf.transform.translation.x;
        p.y = tf.transform.translation.y;
        p.z = tf.transform.translation.z;
        constexpr double minimum_spacing = 0.0015;
        if (marker->points.empty() || pointDistance(marker->points.back(), p) >= minimum_spacing) {
          marker->header.frame_id = planning_frame;
          marker->header.stamp = node->now();
          marker->points.push_back(p);
          marker_pub->publish(*marker);
        }
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Cannot sample the pen pose yet: %s", exception.what());
      }
    });
  (void)trail_timer;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  planning_frame = move_group.getPlanningFrame();
  tool_link = move_group.getEndEffectorLink();

  const auto controller_client =
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node, "/joint_trajectory_controller/follow_joint_trajectory");
  RCLCPP_INFO(node->get_logger(), "Waiting for the trajectory controller...");
  if (!controller_client->wait_for_action_server(90s)) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory controller did not become ready in 90 seconds.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Trajectory controller is ready.");
  
  // Wait an extra 2 seconds for Gazebo physics to stabilize before moving
  RCLCPP_INFO(node->get_logger(), "Waiting 2s for Gazebo physics to stabilize...");
  rclcpp::sleep_for(2s);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  // Ignition's position interface has noticeable tracking lag, but we fixed tolerances.
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::sqrt(0.5);
  orientation.y = std::sqrt(0.5);  // Tool Z axis points toward +X.
  const auto drawing = makeCleanD(
    orientation, plane_x, center_y, bottom_z, height, width);
  const auto & first = drawing.front();
  const auto & last = drawing.back();
  const auto hover =
    makePose(orientation, plane_x - lift, first.position.y, first.position.z);

  RCLCPP_INFO(
    node->get_logger(), "Writing a %.0f x %.0f cm D on X=%.2f m.",
    width * 100.0, height * 100.0, plane_x);
  RCLCPP_INFO(
    node->get_logger(), "Actual green trail: /write_path_marker (%s -> %s).",
    planning_frame.c_str(), tool_link.c_str());

  auto executeCartesian =
    [&node, &move_group](const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const char * description) {
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction =
        move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
      RCLCPP_INFO(
        node->get_logger(), "%s: Cartesian path %.1f%%.", description, fraction * 100.0);
      return fraction > 0.90 &&
             move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    };

  auto moveToPose =
    [&node, &move_group](const geometry_msgs::msg::Pose & target, const char * description) {
      move_group.setPoseTarget(target);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      move_group.clearPoseTargets();
      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "Planning failed: %s.", description);
        return false;
      }
      return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    };

  // Move to a safe bent configuration first to escape the zero-pose singularity.
  std::vector<double> ready_joints = {0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0};
  move_group.setJointValueTarget(ready_joints);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  if (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Moving to 'ready' pose to escape singularity...");
    move_group.execute(joint_plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not plan to ready pose. Proceeding anyway.");
  }
  move_group.clearPoseTargets();

  // The approach is free-space motion. We use joint-space planning (OMPL) because the 
  // orientation changes significantly from the 'ready' pose, which would break Cartesian IK.
  if (!moveToPose(hover, "safe hover pose") ||
      !executeCartesian({first}, "Moving to first drawing point"))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  pen_down.store(true);
  rclcpp::sleep_for(100ms);
  const bool drawn = executeCartesian(drawing, "Drawing letter D");
  rclcpp::sleep_for(100ms);
  pen_down.store(false);
  
  // Ensure the very last point and trail is published when pen is lifted
  marker->header.stamp = node->now();
  marker_pub->publish(*marker);
  if (!drawn) {
    RCLCPP_ERROR(node->get_logger(), "The complete D trajectory could not be executed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const auto retract =
    makePose(orientation, plane_x - lift, last.position.y, last.position.z);
  if (!executeCartesian({retract}, "Lifting the pen")) {
    RCLCPP_WARN(node->get_logger(), "The D is complete, but the pen could not retract cleanly.");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Finished. The green trace remains in RViz. Press Ctrl+C to exit.");
  spinner.join();
  return 0;
}
