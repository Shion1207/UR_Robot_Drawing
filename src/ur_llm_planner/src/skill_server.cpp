// skill_server.cpp
//
// Skill Server cho UR3/UR3e: cung cap service /execute_skill (ur_llm_planner/srv/ExecuteSkill).
// Moi skill duoc thuc hien HOAN TOAN bang MoveIt 2 (MoveGroupInterface):
//   - joint-space planning (OMPL) cho cac chuyen dong lon  -> kiem tra joint limit + collision
//   - Cartesian path (computeCartesianPath, avoid_collisions = true) cho buoc ha/nang thang dung
// LLM khong bao gio sinh quy dao / gia tri khop: no chi chon ten skill + tham so (object, zone),
// con vi tri runtime cua vat den tu camera_perception. config/scene.yaml chi cung cap pose
// spawn/reset va tam cac zone.
//
// Gripper: Robotiq 2F-85 (urdf/ur_robotiq.urdf.xacro), diem tham chieu la grasp_tcp (tam kep).
//   - Ngon kep dong/mo bang gripper_controller (JointTrajectoryController, 6 khop theo he so mimic).
//   - close_gripper: ha grasp_tcp xuong tam vat, kep toi gripper_grasp_position (ngon cach vat
//     ~1 mm), attach vat vao grasp_tcp trong MoveIt va khoa vat vao gripper trong Gazebo bang
//     DetachableJoint (topic /gripper/<vat>/attach) de vat khong truot.
//   - open_gripper: ha vat xuong mat phang ben duoi, mo ngon, nha DetachableJoint, detach MoveIt.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/empty.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ur_llm_planner/srv/execute_skill.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using ExecuteSkill = ur_llm_planner::srv::ExecuteSkill;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace status
{
constexpr const char * SUCCESS = "SUCCESS";
constexpr const char * FAILED = "FAILED";
constexpr const char * INVALID_SKILL = "INVALID_SKILL";
constexpr const char * INVALID_OBJECT = "INVALID_OBJECT";
constexpr const char * INVALID_ZONE = "INVALID_ZONE";
constexpr const char * PLANNING_FAILED = "PLANNING_FAILED";
constexpr const char * EXECUTION_FAILED = "EXECUTION_FAILED";
constexpr const char * OBJECT_NOT_HELD = "OBJECT_NOT_HELD";
constexpr const char * GRIPPER_BUSY = "GRIPPER_BUSY";
constexpr const char * NO_OBJECT_TO_GRASP = "NO_OBJECT_TO_GRASP";
constexpr const char * PERCEPTION_UNAVAILABLE = "PERCEPTION_UNAVAILABLE";
}  // namespace status

struct SkillResult
{
  std::string status;
  std::string message;
  bool ok() const {return status == status::SUCCESS;}
};

static SkillResult success(const std::string & msg) {return {status::SUCCESS, msg};}

using Vec3 = std::array<double, 3>;

class SkillServer
{
public:
  explicit SkillServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    loadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("scene_state", latched);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "skill_server/markers", latched);

    client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    skill_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    gz_client_ = node_->create_client<SetEntityPose>(
      "/world/" + gz_world_name_ + "/set_pose", rmw_qos_profile_services_default, client_group_);
    for (const auto & kv : objects_) {
      gz_attach_pubs_[kv.first] = node_->create_publisher<std_msgs::msg::Empty>(
        "/gripper/" + kv.first + "/attach", 10);
      gz_detach_pubs_[kv.first] = node_->create_publisher<std_msgs::msg::Empty>(
        "/gripper/" + kv.first + "/detach", 10);
    }
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, gripper_action_, client_group_);
  }

  // Tao MoveGroupInterface (cho toi khi move_group san sang), dung planning scene, mo service.
  void initialize()
  {
    RCLCPP_INFO(node_->get_logger(), "Dang ket noi toi move_group (group '%s')...",
      planning_group_.c_str());
    move_group_ = std::make_shared<MoveGroupInterface>(node_, planning_group_, tf_buffer_);
    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->startStateMonitor();
    // Scene cuc bo de loc nghiem IK: tu va cham (theo SRDF) + va cham voi ban.
    ik_check_scene_ = std::make_shared<planning_scene::PlanningScene>(
      move_group_->getRobotModel());

    setupPlanningScene();
    const SkillResult opened = commandGripper(gripper_open_position_, "mo gripper");
    if (!opened.ok()) {
      RCLCPP_WARN(node_->get_logger(), "%s", opened.message.c_str());
    }
    resetGazeboObjects();

    rclcpp::SubscriptionOptions camera_options;
    camera_options.callback_group = skill_group_;
    camera_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "camera/object_poses", rclcpp::QoS(1).transient_local().reliable(),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {onCameraPoses(msg);},
      camera_options);
    publishMarkers();
    publishSceneState();

    service_ = node_->create_service<ExecuteSkill>(
      "execute_skill",
      [this](const std::shared_ptr<ExecuteSkill::Request> req,
      std::shared_ptr<ExecuteSkill::Response> res) {handleRequest(req, res);},
      rmw_qos_profile_services_default, skill_group_);

    RCLCPP_INFO(node_->get_logger(),
      "Skill server san sang. Service: /execute_skill | skills: home, pick, place, "
      "move_above, move_to_zone, open_gripper, close_gripper");
  }

private:
  // ------------------------------------------------------------------ parameters
  void loadParameters()
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "ur_manipulator");
    ee_link_ = node_->declare_parameter<std::string>("ee_link", "grasp_tcp");
    world_frame_ = node_->declare_parameter<std::string>("world_frame", "world");
    gz_world_name_ = node_->declare_parameter<std::string>("gz_world_name", "pick_place_world");
    gz_sync_ = node_->declare_parameter<bool>("gz_sync", true);

    velocity_scaling_ = node_->declare_parameter<double>("velocity_scaling", 0.3);
    acceleration_scaling_ = node_->declare_parameter<double>("acceleration_scaling", 0.3);
    planning_time_ = node_->declare_parameter<double>("planning_time", 5.0);
    planning_retries_ = node_->declare_parameter<int>("planning_retries", 3);
    eef_step_ = node_->declare_parameter<double>("eef_step", 0.005);
    min_cartesian_fraction_ = node_->declare_parameter<double>("min_cartesian_fraction", 0.98);

    approach_height_ = node_->declare_parameter<double>("approach_height", 0.12);
    place_clearance_ = node_->declare_parameter<double>("place_clearance", 0.003);

    gripper_action_ = node_->declare_parameter<std::string>(
      "gripper_action", "/gripper_controller/follow_joint_trajectory");
    gripper_joints_ = node_->declare_parameter<std::vector<std::string>>(
      "gripper_joints", {"robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"});
    // He so mimic trong robotiq_2f_85_macro.urdf.xacro, theo thu tu gripper_joints
    gripper_multipliers_ = node_->declare_parameter<std::vector<double>>(
      "gripper_mimic_multipliers", {1.0, -1.0, 1.0, -1.0, -1.0, 1.0});
    if (gripper_joints_.size() != gripper_multipliers_.size()) {
      throw std::runtime_error("gripper_joints va gripper_mimic_multipliers phai cung so phan tu");
    }
    gripper_open_position_ = node_->declare_parameter<double>("gripper_open_position", 0.0);
    // 0.43 rad: 2 ngon cach nhau 42 mm, sat khoi 40 mm ma khong ep vao vat
    gripper_grasp_position_ = node_->declare_parameter<double>("gripper_grasp_position", 0.43);
    gripper_motion_time_ = node_->declare_parameter<double>("gripper_motion_time", 1.0);
    touch_links_ = node_->declare_parameter<std::vector<std::string>>(
      "touch_links", {"grasp_tcp", "robotiq_85_base_link",
        "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
        "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
        "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
        "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"});

    home_joints_ = node_->declare_parameter<std::vector<double>>(
      "home_joints", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});

    const auto table_size = node_->declare_parameter<std::vector<double>>(
      "table_size", {0.90, 1.00, 0.75});
    const auto table_center = node_->declare_parameter<std::vector<double>>(
      "table_center", {0.15, 0.0, -0.375});
    table_size_ = toVec3(table_size, "table_size");
    table_center_ = toVec3(table_center, "table_center");
    table_padding_ = node_->declare_parameter<double>("table_padding", 0.01);

    cube_size_ = node_->declare_parameter<double>("cube_size", 0.04);
    zone_size_ = node_->declare_parameter<double>("zone_size", 0.08);
    camera_required_ = node_->declare_parameter<bool>("camera_required", true);

    object_names_ = node_->declare_parameter<std::vector<std::string>>(
      "object_names", std::vector<std::string>{});
    for (const auto & name : object_names_) {
      const auto p = node_->declare_parameter<std::vector<double>>(
        "objects." + name + ".position", std::vector<double>{});
      objects_[name] = toVec3(p, "objects." + name + ".position");
    }

    const auto zone_names = node_->declare_parameter<std::vector<std::string>>(
      "zone_names", std::vector<std::string>{});
    for (const auto & name : zone_names) {
      const auto p = node_->declare_parameter<std::vector<double>>(
        "zones." + name + ".position", std::vector<double>{});
      zones_[name] = toVec3(p, "zones." + name + ".position");
    }

    if (objects_.empty() || zones_.empty()) {
      throw std::runtime_error(
              "Chua khai bao object_names / zone_names. Hay nap config/scene.yaml cho node.");
    }
  }

  static Vec3 toVec3(const std::vector<double> & v, const std::string & name)
  {
    if (v.size() != 3) {
      throw std::runtime_error("Parameter '" + name + "' phai la mang 3 phan tu [x, y, z]");
    }
    return {v[0], v[1], v[2]};
  }

  // ------------------------------------------------------------------ planning scene
  moveit_msgs::msg::CollisionObject makeBox(
    const std::string & id, const Vec3 & center, const Vec3 & size) const
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = world_frame_;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size[0], size[1], size[2]};
    geometry_msgs::msg::Pose pose;
    pose.position.x = center[0];
    pose.position.y = center[1];
    pose.position.z = center[2];
    pose.orientation.w = 1.0;
    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
  }

  void setupPlanningScene()
  {
    // Neu skill_server khoi dong lai trong khi move_group van chay, go cac vat con dang attach.
    for (const auto & kv : planning_scene_.getAttachedObjects()) {
      move_group_->detachObject(kv.first);
    }

    std::vector<moveit_msgs::msg::CollisionObject> objs;
    // Ban ha thap table_padding de de robot (dat tren mat ban) khong bi coi la va cham.
    Vec3 table_center = table_center_;
    table_center[2] -= table_padding_ / 2.0;
    Vec3 table_size = table_size_;
    table_size[2] -= table_padding_;
    objs.push_back(makeBox("table", table_center, table_size));
    ik_check_scene_->processCollisionObjectMsg(objs.back());
    for (const auto & kv : objects_) {
      objs.push_back(makeBox(kv.first, kv.second, {cube_size_, cube_size_, cube_size_}));
    }
    if (!planning_scene_.applyCollisionObjects(objs)) {
      RCLCPP_WARN(node_->get_logger(), "Khong the them collision objects vao planning scene");
    }
    held_object_.clear();
  }

  // ------------------------------------------------------------------ Gazebo sync
  void setGazeboPose(const std::string & name, const Vec3 & p)
  {
    if (!gz_sync_) {
      return;
    }
    if (!gz_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Service %s chua san sang - Gazebo se khong cap nhat vi tri vat",
        gz_client_->get_service_name());
      return;
    }
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = name;
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    req->pose.position.x = p[0];
    req->pose.position.y = p[1];
    req->pose.position.z = p[2];
    req->pose.orientation.w = 1.0;
    gz_client_->async_send_request(req);
  }

  // Khoa / nha vat vao gripper trong Gazebo (DetachableJoint, bridge bang ros_gz_bridge).
  void setGazeboAttached(const std::string & name, bool attached)
  {
    if (!gz_sync_) {
      return;
    }
    const auto & pub = attached ? gz_attach_pubs_.at(name) : gz_detach_pubs_.at(name);
    if (pub->get_subscription_count() == 0) {
      RCLCPP_WARN(node_->get_logger(),
        "Chua co bridge cho %s - Gazebo se khong %s vat", pub->get_topic_name(),
        attached ? "khoa" : "nha");
    }
    pub->publish(std_msgs::msg::Empty());
  }

  bool waitForGazeboBridges(std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      bool ready = true;
      for (const auto & kv : gz_detach_pubs_) {
        ready = ready && kv.second->get_subscription_count() > 0 &&
          gz_attach_pubs_.at(kv.first)->get_subscription_count() > 0;
      }
      if (ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  void resetGazeboObjects()
  {
    if (!gz_sync_) {
      return;
    }
    // DetachableJoint tu khoa moi vat vao gripper luc khoi dong -> nha het truoc khi dat lai.
    if (!waitForGazeboBridges(std::chrono::seconds(10))) {
      RCLCPP_WARN(node_->get_logger(),
        "Khong thay bridge /gripper/<vat>/attach|detach. Robot van chay duoc nhung vat trong "
        "Gazebo se khong di theo gripper.");
    }
    for (const auto & kv : objects_) {
      setGazeboAttached(kv.first, false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!gz_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_WARN(node_->get_logger(),
        "Khong thay service %s - khong dat lai duoc vi tri ban dau cua vat trong Gazebo.",
        gz_client_->get_service_name());
      return;
    }
    for (const auto & kv : objects_) {
      setGazeboPose(kv.first, kv.second);
    }
  }

  std::optional<Vec3> toolPosition()
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(world_frame_, ee_link_, tf2::TimePointZero);
      return Vec3{tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Khong lay duoc TF %s -> %s: %s", world_frame_.c_str(), ee_link_.c_str(), e.what());
      return std::nullopt;
    }
  }

  // ------------------------------------------------------------------ state helpers
  bool validObject(const std::string & name) const {return objects_.count(name) > 0;}
  bool validZone(const std::string & name) const {return zones_.count(name) > 0;}

  std::string zoneOf(const Vec3 & p) const
  {
    const double half = zone_size_ / 2.0 + 0.01;
    for (const auto & kv : zones_) {
      if (std::fabs(p[0] - kv.second[0]) <= half && std::fabs(p[1] - kv.second[1]) <= half) {
        return kv.first;
      }
    }
    return "";
  }

  // Do cao mat tren cao nhat trong vung (cho phep xep chong khoi len nhau).
  double zoneSurfaceHeight(const std::string & zone, const std::string & exclude) const
  {
    double surface = zones_.at(zone)[2];
    for (const auto & kv : objects_) {
      if (kv.first == exclude) {
        continue;
      }
      if (zoneOf(kv.second) == zone) {
        surface = std::max(surface, kv.second[2] + cube_size_ / 2.0);
      }
    }
    return surface;
  }

  geometry_msgs::msg::Pose toolDownPose(double x, double y, double z) const
  {
    // grasp_tcp (cung huong tool0) chi thang xuong mat ban (RPY = [pi, 0, -pi/2]), cung huong
    // voi tu the home de co tay khong phai xoay nhieu. 2 ngon kep khep theo truc y cua world.
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = -M_SQRT1_2;
    pose.orientation.y = M_SQRT1_2;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  // Camera la nguon toa do runtime. YAML chi dung de spawn/reset vat luc khoi dong.
  void onCameraPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->header.frame_id != world_frame_ || msg->poses.size() != object_names_.size()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "camera/object_poses sai frame hoac sai so object");
      return;
    }

    bool all_visible = true;
    bool changed = false;
    for (std::size_t i = 0; i < object_names_.size(); ++i) {
      const auto & pose = msg->poses[i];
      const bool visible = std::isfinite(pose.position.x) && std::isfinite(pose.position.y);
      object_visible_[object_names_[i]] = visible;
      if (!visible) {
        all_visible = false;
        continue;
      }
      if (object_names_[i] == held_object_) {
        continue;
      }
      const Vec3 measured{pose.position.x, pose.position.y, cube_size_ / 2.0};
      const Vec3 & old = objects_.at(object_names_[i]);
      const double moved = std::hypot(measured[0] - old[0], measured[1] - old[1]);
      if (moved < 0.002) {
        continue;
      }
      objects_[object_names_[i]] = measured;
      planning_scene_.applyCollisionObject(
        makeBox(object_names_[i], measured, {cube_size_, cube_size_, cube_size_}));
      changed = true;
    }

    ++camera_sequence_;
    camera_ready_ = camera_ready_ || all_visible;
    camera_all_visible_ = all_visible;
    if (changed || camera_sequence_ % 10 == 0) {
      publishSceneState();
    }
  }

  void publishSceneState()
  {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << "{\"held_object\": " << (held_object_.empty() ? "null" : "\"" + held_object_ + "\"");
    ss << ", \"source\": \"camera\", \"camera_ready\": "
       << (camera_ready_ ? "true" : "false") << ", \"all_visible\": "
       << (camera_all_visible_ ? "true" : "false") << ", \"camera_sequence\": " << camera_sequence_;
    ss << ", \"objects\": {";
    bool first = true;
    for (const auto & kv : objects_) {
      const std::string zone = kv.first == held_object_ ? "" : zoneOf(kv.second);
      ss << (first ? "" : ", ") << "\"" << kv.first << "\": {\"position\": [" << kv.second[0]
         << ", " << kv.second[1] << ", " << kv.second[2] << "], \"in_zone\": "
         << (zone.empty() ? "null" : "\"" + zone + "\"") << ", \"visible\": "
         << (object_visible_[kv.first] ? "true" : "false") << "}";
      first = false;
    }
    ss << "}, \"zones\": [";
    first = true;
    for (const auto & kv : zones_) {
      ss << (first ? "" : ", ") << "\"" << kv.first << "\"";
      first = false;
    }
    ss << "]}";
    std_msgs::msg::String msg;
    msg.data = ss.str();
    state_pub_->publish(msg);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto & kv : zones_) {
      visualization_msgs::msg::Marker pad;
      pad.header.frame_id = world_frame_;
      pad.ns = "zones";
      pad.id = id++;
      pad.type = visualization_msgs::msg::Marker::CUBE;
      pad.pose.position.x = kv.second[0];
      pad.pose.position.y = kv.second[1];
      pad.pose.position.z = kv.second[2] + 0.001;
      pad.pose.orientation.w = 1.0;
      pad.scale.x = zone_size_;
      pad.scale.y = zone_size_;
      pad.scale.z = 0.002;
      pad.color.g = 0.8;
      pad.color.b = 0.3;
      pad.color.a = 0.5;
      arr.markers.push_back(pad);

      visualization_msgs::msg::Marker label = pad;
      label.ns = "zone_labels";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.text = kv.first;
      label.pose.position.z += 0.08;
      label.scale.z = 0.03;
      label.color.r = label.color.g = label.color.b = 1.0;
      label.color.a = 1.0;
      arr.markers.push_back(label);
    }
    marker_pub_->publish(arr);
  }

  // ------------------------------------------------------------------ motion primitives
  SkillResult planAndExecute(const std::string & what)
  {
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= planning_retries_ && !planned; ++attempt) {
      move_group_->setStartStateToCurrentState();
      planned = static_cast<bool>(move_group_->plan(plan));
      if (!planned) {
        RCLCPP_WARN(node_->get_logger(), "Lap ke hoach '%s' that bai (lan %d/%d)",
          what.c_str(), attempt, planning_retries_);
      }
    }
    if (!planned) {
      return {status::PLANNING_FAILED, "MoveIt khong tim duoc quy dao hop le cho: " + what};
    }
    if (!static_cast<bool>(move_group_->execute(plan))) {
      return {status::EXECUTION_FAILED, "Thuc thi quy dao that bai: " + what};
    }
    return success(what);
  }

  // Cau hinh khop "de chiu" cho thao tac tren ban: |pan| <= pi, elbow-up, co tay huong xuong.
  // Loai bo cac nghiem IK xoan (vd: pan = 4.4 rad) - van hop le nhung sau do khong the
  // ha/nang thang dung bang Cartesian path.
  static bool preferredConfiguration(const double * v)
  {
    return std::fabs(v[0]) <= M_PI &&   // shoulder_pan
           v[1] <= 0.0 && v[1] >= -M_PI &&  // shoulder_lift: canh tay tren huong len
           v[2] >= 0.0 && v[2] <= M_PI &&   // elbow
           v[4] <= 0.0 && v[4] >= -M_PI;    // wrist_2
  }

  // Giai IK (co rang buoc cau hinh + khong va cham), sau do de OMPL lap ke hoach trong khong
  // gian khop toi nghiem do -> quy dao duoc kiem tra joint limit, self-collision va va cham moi
  // truong. Neu OMPL that bai (vd: nghiem cham vat tren ban) thi giai lai IK tu seed khac.
  SkillResult moveToPose(const geometry_msgs::msg::Pose & pose, const std::string & what)
  {
    const auto current = move_group_->getCurrentState(2.0);
    if (!current) {
      return {status::FAILED, "Khong doc duoc trang thai hien tai cua robot"};
    }
    const auto * jmg = current->getJointModelGroup(planning_group_);
    const moveit::core::GroupStateValidityCallbackFn check =
      [this](moveit::core::RobotState * state, const moveit::core::JointModelGroup * group,
        const double * v) {
        if (!preferredConfiguration(v)) {
          return false;
        }
        state->setJointGroupPositions(group, v);
        state->update();
        return !ik_check_scene_->isStateColliding(*state);
      };

    SkillResult result{status::PLANNING_FAILED,
      "Khong tim duoc nghiem IK hop le cho " + what +
      " (ngoai tam voi / vuot joint limit / va cham)"};
    moveit::core::RobotState state(*current);
    constexpr int kIkAttempts = 22;  // seed: trang thai hien tai, home, roi 20 seed ngau nhien
    for (int attempt = 0; attempt < kIkAttempts; ++attempt) {
      if (attempt == 0) {
        state = *current;
      } else if (attempt == 1) {
        state.setJointGroupPositions(jmg, home_joints_);
      } else {
        state.setToRandomPositions(jmg);
      }
      if (!state.setFromIK(jmg, pose, ee_link_, attempt < 2 ? 0.1 : 0.05, check)) {
        continue;
      }
      std::vector<double> joints;
      state.copyJointGroupPositions(jmg, joints);
      move_group_->setJointValueTarget(joints);
      result = planAndExecute(what);
      if (result.status != status::PLANNING_FAILED) {
        return result;  // SUCCESS hoac EXECUTION_FAILED: khong thu nghiem IK khac
      }
      RCLCPP_WARN(node_->get_logger(), "'%s': thu nghiem IK khac (lan %d)", what.c_str(),
        attempt + 1);
    }
    return result;
  }

  // Chuyen dong thang (Cartesian) - dung cho ha/nang theo phuong thang dung.
  SkillResult moveLinear(const geometry_msgs::msg::Pose & target, const std::string & what)
  {
    move_group_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
      {target}, eef_step_, 0.0, trajectory, true);
    if (fraction < min_cartesian_fraction_) {
      std::ostringstream ss;
      ss << "Cartesian path '" << what << "' chi dat " << fraction * 100.0
         << "% (co the do va cham / vuot gioi han khop / IK that bai)";
      return {status::PLANNING_FAILED, ss.str()};
    }
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (!static_cast<bool>(move_group_->execute(plan))) {
      return {status::EXECUTION_FAILED, "Thuc thi Cartesian path that bai: " + what};
    }
    return success(what);
  }

  // Dua ngon kep toi goc `position` cua robotiq_85_left_knuckle_joint (0 = mo, 0.79 = dong).
  SkillResult commandGripper(
    double position, const std::string & what, bool contact_is_success = false)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return {status::FAILED, "Action " + gripper_action_ + " chua san sang "
        "(gripper_controller chua chay?)"};
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = gripper_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const double m : gripper_multipliers_) {
      point.positions.push_back(m * position);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(gripper_motion_time_);
    goal.trajectory.points.push_back(point);

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return {status::EXECUTION_FAILED, "Khong gui duoc lenh toi gripper: " + what};
    }
    const auto handle = goal_future.get();
    if (!handle) {
      return {status::EXECUTION_FAILED, "gripper_controller tu choi lenh: " + what};
    }
    auto result_future = gripper_client_->async_get_result(handle);
    const auto timeout = std::chrono::duration<double>(gripper_motion_time_ + 5.0);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      // Khi kep vat, ngon kep co the cham vat truoc khi dat chinh xac joint target.
      // JointTrajectoryController khi do co the giu goal ACTIVE vo han thay vi tra
      // SUCCESS. Day la ket qua mong doi cua mot thao tac grasp, khong phai loi.
      if (contact_is_success) {
        (void)gripper_client_->async_cancel_goal(handle);
        RCLCPP_WARN(
          node_->get_logger(),
          "Gripper bi vat chan truoc joint target khi '%s'; coi nhu da kep va huy goal",
          what.c_str());
        return success(what + " (da cham vat)");
      }
      return {status::EXECUTION_FAILED, "Het thoi gian cho gripper: " + what};
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      return {status::EXECUTION_FAILED,
        "Gripper that bai (" + what + "): " + result.result->error_string};
    }
    return success(what);
  }

  // ------------------------------------------------------------------ skills
  SkillResult skillHome()
  {
    if (!move_group_->setJointValueTarget(home_joints_)) {
      return {status::FAILED, "home_joints khong hop le (sai so khop hoac vuot joint limit)"};
    }
    return planAndExecute("home");
  }

  SkillResult skillMoveAbove(const std::string & object)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (object == held_object_) {
      return {status::FAILED, "Khong the di toi phia tren '" + object + "' vi dang cam no"};
    }
    const Vec3 & p = objects_.at(object);
    return moveToPose(
      toolDownPose(p[0], p[1], p[2] + approach_height_), "move_above(" + object + ")");
  }

  SkillResult skillMoveToZone(const std::string & zone)
  {
    if (!validZone(zone)) {
      return {status::INVALID_ZONE, "Vung khong ton tai: '" + zone + "'"};
    }
    const Vec3 & z = zones_.at(zone);
    const double surface = zoneSurfaceHeight(zone, held_object_);
    return moveToPose(
      toolDownPose(z[0], z[1], surface + cube_size_ / 2.0 + approach_height_),
      "move_to_zone(" + zone + ")");
  }

  // Mat phang nam ngay duoi (x, y): mat ban/vung (z = 0) hoac dinh vat cao nhat ben duoi.
  double surfaceBelow(double x, double y, const std::string & exclude) const
  {
    double surface = 0.0;
    for (const auto & kv : objects_) {
      if (kv.first == exclude) {
        continue;
      }
      const Vec3 & p = kv.second;
      if (std::fabs(p[0] - x) < cube_size_ && std::fabs(p[1] - y) < cube_size_) {
        surface = std::max(surface, p[2] + cube_size_ / 2.0);
      }
    }
    return surface;
  }

  // Ha grasp_tcp xuong tam vat ngay ben duoi, dong ngon, khoa vat, nang ve do cao cu.
  SkillResult skillCloseGripper(const std::string & expected = "")
  {
    if (!held_object_.empty()) {
      return {status::GRIPPER_BUSY, "Gripper dang giu '" + held_object_ + "'"};
    }
    const auto tool = toolPosition();
    if (!tool) {
      return {status::FAILED, "Khong xac dinh duoc vi tri " + ee_link_};
    }
    // Vat cao nhat nam thang hang ben duoi grasp_tcp.
    std::string target;
    double best_z = -1e9;
    for (const auto & kv : objects_) {
      const Vec3 & p = kv.second;
      const double dxy = std::hypot(p[0] - (*tool)[0], p[1] - (*tool)[1]);
      const double gap = (*tool)[2] - p[2];
      if (dxy < cube_size_ / 2.0 && gap > -0.005 && gap < approach_height_ + 0.03 &&
        p[2] > best_z)
      {
        target = kv.first;
        best_z = p[2];
      }
    }
    if (target.empty()) {
      return {status::NO_OBJECT_TO_GRASP, "Khong co vat nao ngay duoi gripper"};
    }
    if (!expected.empty() && target != expected) {
      return {status::FAILED, "Vat ngay duoi gripper la '" + target + "', khong phai '" +
        expected + "' (co vat khac xep chong len tren?)"};
    }

    SkillResult r = commandGripper(gripper_open_position_, "mo gripper");
    if (!r.ok()) {return r;}
    const Vec3 & p = objects_.at(target);
    r = moveLinear(toolDownPose(p[0], p[1], p[2]), "ha xuong gap " + target);
    if (!r.ok()) {return r;}
    r = commandGripper(gripper_grasp_position_, "kep " + target, true);
    if (!r.ok()) {return r;}

    if (!move_group_->attachObject(target, ee_link_, touch_links_)) {
      return {status::FAILED, "Khong attach duoc '" + target + "' vao " + ee_link_};
    }
    setGazeboAttached(target, true);
    held_object_ = target;

    r = moveLinear(toolDownPose(p[0], p[1], (*tool)[2]), "nang " + target + " len");
    if (!r.ok()) {return r;}
    return success("Da gap '" + target + "'");
  }

  // Ha vat dang cam xuong mat phang ben duoi, mo ngon, nha vat, nang ve do cao cu.
  SkillResult skillOpenGripper()
  {
    if (held_object_.empty()) {
      const SkillResult r = commandGripper(gripper_open_position_, "mo gripper");
      return r.ok() ? success("Gripper da mo (khong giu vat nao)") : r;
    }
    const std::string object = held_object_;
    const auto tool = toolPosition();
    if (!tool) {
      return {status::FAILED, "Khong xac dinh duoc vi tri " + ee_link_};
    }
    const double surface = surfaceBelow((*tool)[0], (*tool)[1], object);
    SkillResult r = moveLinear(
      toolDownPose((*tool)[0], (*tool)[1], surface + cube_size_ / 2.0 + place_clearance_),
      "ha " + object + " xuong");
    if (!r.ok()) {return r;}
    r = commandGripper(gripper_open_position_, "tha " + object);
    if (!r.ok()) {return r;}

    const auto release = toolPosition();
    setGazeboAttached(object, false);
    move_group_->detachObject(object);
    held_object_.clear();
    const Vec3 at = release ? *release : *tool;
    // Vat nam dung tren mat phang ben duoi (bo qua khe ho place_clearance).
    objects_[object] = {at[0], at[1], surface + cube_size_ / 2.0};
    setGazeboPose(object, objects_[object]);
    // Dong bo lai vi tri vat trong planning scene cua MoveIt.
    planning_scene_.applyCollisionObject(
      makeBox(object, objects_[object], {cube_size_, cube_size_, cube_size_}));

    r = moveLinear(toolDownPose((*tool)[0], (*tool)[1], (*tool)[2]), "nang gripper len");
    if (!r.ok()) {return r;}
    return success("Da tha '" + object + "'");
  }

  SkillResult skillPick(const std::string & object)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (!held_object_.empty()) {
      return {status::GRIPPER_BUSY,
        "Dang giu '" + held_object_ + "', phai place truoc khi pick '" + object + "'"};
    }
    SkillResult r = skillMoveAbove(object);
    if (!r.ok()) {return r;}
    r = skillCloseGripper(object);
    if (!r.ok()) {return r;}
    return success("pick(" + object + ") thanh cong");
  }

  SkillResult skillPlace(const std::string & object, const std::string & zone)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (!validZone(zone)) {
      return {status::INVALID_ZONE, "Vung khong ton tai: '" + zone + "'"};
    }
    if (held_object_ != object) {
      return {status::OBJECT_NOT_HELD,
        "Gripper khong giu '" + object + "' (dang giu: '" +
        (held_object_.empty() ? "khong co gi" : held_object_) + "')"};
    }
    SkillResult r = skillMoveToZone(zone);
    if (!r.ok()) {return r;}
    r = skillOpenGripper();
    if (!r.ok()) {return r;}
    return success("place(" + object + ", " + zone + ") thanh cong");
  }

  // ------------------------------------------------------------------ service
  SkillResult dispatch(const std::string & skill, const std::string & object,
    const std::string & zone)
  {
    if (skill == "home") {return skillHome();}
    if (skill == "pick") {return skillPick(object);}
    if (skill == "place") {return skillPlace(object, zone);}
    if (skill == "move_above") {return skillMoveAbove(object);}
    if (skill == "move_to_zone") {return skillMoveToZone(zone);}
    if (skill == "open_gripper") {return skillOpenGripper();}
    if (skill == "close_gripper") {return skillCloseGripper();}
    return {status::INVALID_SKILL, "Skill khong nam trong danh sach cho phep: '" + skill + "'"};
  }

  void handleRequest(
    const std::shared_ptr<ExecuteSkill::Request> req,
    std::shared_ptr<ExecuteSkill::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), ">> skill=%s object=%s zone=%s",
      req->skill.c_str(), req->object.c_str(), req->zone.c_str());
    SkillResult r;
    try {
      if (camera_required_ && !camera_ready_ && req->skill != "home") {
        r = {status::PERCEPTION_UNAVAILABLE,
          "Camera chua nhan dien du 5 vat; tu choi thao tac dung vi tri vat"};
      } else {
        r = dispatch(req->skill, req->object, req->zone);
      }
    } catch (const std::exception & e) {
      r = {status::FAILED, std::string("Exception: ") + e.what()};
    }
    // Vat dang cam nam giua 2 ngon: tam vat trung voi grasp_tcp.
    if (!held_object_.empty()) {
      if (const auto tool = toolPosition()) {
        objects_[held_object_] = *tool;
      }
    }
    publishSceneState();
    res->status = r.status;
    res->message = r.message;
    if (r.ok()) {
      RCLCPP_INFO(node_->get_logger(), "<< %s: %s", r.status.c_str(), r.message.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "<< %s: %s", r.status.c_str(), r.message.c_str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  planning_scene::PlanningScenePtr ik_check_scene_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr skill_group_;
  rclcpp::Service<ExecuteSkill>::SharedPtr service_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr camera_pose_sub_;
  rclcpp::Client<SetEntityPose>::SharedPtr gz_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> gz_attach_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> gz_detach_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string planning_group_, ee_link_, world_frame_, gz_world_name_;
  bool gz_sync_ = true;
  bool camera_required_ = true, camera_ready_ = false, camera_all_visible_ = false;
  std::size_t camera_sequence_ = 0;
  double velocity_scaling_ = 0.3, acceleration_scaling_ = 0.3, planning_time_ = 5.0;
  int planning_retries_ = 3;
  double eef_step_ = 0.005, min_cartesian_fraction_ = 0.98;
  double approach_height_ = 0.12, place_clearance_ = 0.003;
  std::vector<double> home_joints_;
  Vec3 table_size_{}, table_center_{};
  double table_padding_ = 0.01;
  double cube_size_ = 0.04, zone_size_ = 0.08;

  std::string gripper_action_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> gripper_multipliers_;
  double gripper_open_position_ = 0.0, gripper_grasp_position_ = 0.43, gripper_motion_time_ = 1.0;
  std::vector<std::string> touch_links_;

  std::vector<std::string> object_names_;
  std::map<std::string, bool> object_visible_;
  std::map<std::string, Vec3> objects_;
  std::map<std::string, Vec3> zones_;
  std::string held_object_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("skill_server");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int rc = 0;
  try {
    SkillServer server(node);
    server.initialize();
    spinner.join();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    rc = 1;
    rclcpp::shutdown();
    spinner.join();
  }
  rclcpp::shutdown();
  return rc;
}
