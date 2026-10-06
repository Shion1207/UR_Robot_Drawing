"""UR3/UR3e + Robotiq 2F-85 + Gazebo (ban, 3 khoi, 3 vung) + MoveIt 2 + skill_server.

Vi du:
  ros2 launch ur_llm_planner sim.launch.py                 # UR3e
  ros2 launch ur_llm_planner sim.launch.py ur_type:=ur3
  ros2 launch ur_llm_planner sim.launch.py gazebo_gui:=false launch_rviz:=false

Sau do chay LLM planner o terminal khac:
  ros2 run ur_llm_planner llm_planner_node.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

WORLD_NAME = "pick_place_world"


def package_share_with_file(package, *required_path):
    """Bo qua overlay bi hong (symlink dut) va tim ban package co file that."""
    candidates = []
    try:
        candidates.append(get_package_share_directory(package))
    except Exception:  # package co the chi ton tai o prefix sau
        pass
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share", package)
        if share not in candidates:
            candidates.append(share)
    for share in candidates:
        if os.path.isfile(os.path.join(share, *required_path)):
            return share
    raise FileNotFoundError(f"Khong tim thay {package}/{'/'.join(required_path)}")


def load_yaml(package, *path):
    with open(os.path.join(package_share_with_file(package, *path), *path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    startup_delay = LaunchConfiguration("startup_delay")

    pkg_share = FindPackageShare("ur_llm_planner")
    world_file = PathJoinSubstitution([pkg_share, "worlds", "pick_place.sdf"])
    scene_config = PathJoinSubstitution([pkg_share, "config", "scene.yaml"])
    gripper_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "gripper_controller.yaml"])
    # Can kinematics.yaml de skill_server tu giai IK (chon cau hinh khop hop ly)
    moveit_share = package_share_with_file("ur_moveit_config", "config", "kinematics.yaml")
    # Workspace cu con resource marker ur_moveit_config nhung symlink source da dut. Loai rieng
    # prefix hong khoi moi truong cua xacro de $(find ur_moveit_config) roi xuong /opt/ros.
    ament_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    valid_prefixes = [p for p in ament_prefixes if not (
        os.path.isfile(os.path.join(
            p, "share", "ament_index", "resource_index", "packages", "ur_moveit_config"))
        and not os.path.isfile(os.path.join(
            p, "share", "ur_moveit_config", "srdf", "ur_macro.srdf.xacro")))]
    clean_ament_path = SetEnvironmentVariable("AMENT_PREFIX_PATH", os.pathsep.join(valid_prefixes))
    kinematics_config = PathJoinSubstitution(
        [moveit_share, "config", "kinematics.yaml"])

    object_names = load_yaml("ur_llm_planner", "config", "scene.yaml")[
        "/**"]["ros__parameters"]["object_names"]

    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3", "ur3e"],
                              description="Loai robot: ur3 hoac ur3e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("startup_delay", default_value="8.0",
                              description="Giay cho Gazebo/controller truoc khi chay skill_server"),
    ]

    # 1) Gazebo + ros2_control (dung lai launch cua ur_simulation_gz, thay world va URDF co gripper)
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch",
                                  "ur_sim_control.launch.py"])),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur_llm_planner",
            "description_file": "ur_robotiq.urdf.xacro",
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager",
                   "--param-file", gripper_controller_config,
                   "--controller-manager-timeout", "120"],
        output="screen",
    )

    # 2) MoveIt 2: move_group + RViz voi URDF/SRDF co gripper (cac cau hinh con lai cua
    #    ur_moveit_config giu nguyen)
    robot_description = {"robot_description": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "urdf", "ur_robotiq.urdf.xacro"]),
        " name:=ur ur_type:=", ur_type, " safety_limits:=true use_fake_hardware:=true",
    ]), value_type=str)}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "srdf", "ur_robotiq.srdf.xacro"]), " name:=ur",
    ]), value_type=str)}

    ompl = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl.update(load_yaml("ur_moveit_config", "config", "ompl_planning.yaml"))

    # scaled_joint_trajectory_controller khong chay trong Gazebo
    controllers = load_yaml("ur_moveit_config", "config", "controllers.yaml")
    controllers["scaled_joint_trajectory_controller"]["default"] = False
    controllers["joint_trajectory_controller"]["default"] = True

    moveit_params = [
        robot_description,
        robot_description_semantic,
        kinematics_config,
        {"robot_description_planning":
            load_yaml("ur_moveit_config", "config", "joint_limits.yaml")},
        {"move_group": ompl},
        {"use_sim_time": True},
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params + [
            {"publish_robot_description_semantic": True},
            {"moveit_simple_controller_manager": controllers,
             "moveit_controller_manager":
                 "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"moveit_manage_controllers": False,
             "trajectory_execution.allowed_execution_duration_scaling": 1.2,
             "trajectory_execution.allowed_goal_duration_margin": 0.5,
             "trajectory_execution.allowed_start_tolerance": 0.01,
             "trajectory_execution.execution_duration_monitoring": False},
            {"publish_planning_scene": True,
             "publish_geometry_updates": True,
             "publish_state_updates": True,
             "publish_transforms_updates": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [moveit_share, "rviz", "view_robot.rviz"])],
        parameters=moveit_params,
        condition=IfCondition(launch_rviz),
    )

    # 3) Bridge ROS 2 <-> Gazebo: camera, set_pose va gan/nha vat (DetachableJoint)
    gripper_topics = [f"/gripper/{name}/{action}@std_msgs/msg/Empty]ignition.msgs.Empty"
                      for name in object_names for action in ("attach", "detach")]
    camera_topics = [
        "/overhead_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
        "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
    ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_gripper_bridge",
        arguments=[f"/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose"]
        + gripper_topics + camera_topics,
        output="screen",
    )

    # 4) Skill server: thuc thi robot skill bang MoveIt 2
    skill_server = Node(
        package="ur_llm_planner",
        executable="skill_server",
        name="skill_server",
        output="screen",
        parameters=[scene_config, kinematics_config,
                    {"use_sim_time": True, "gz_world_name": WORLD_NAME}],
    )

    # 5) Perception: anh RGB -> pose 5 cube + trang thai zone. Skill server dung pose camera
    #    cho muc tieu pick; LLM chi nhan ten object/zone, khong nhan toa do.
    camera_perception = Node(
        package="ur_llm_planner",
        executable="camera_perception_node.py",
        name="camera_perception",
        output="screen",
        parameters=[scene_config, {"use_sim_time": True}],
    )

    return LaunchDescription(declared_arguments + [
        clean_ament_path,
        ur_control,
        gripper_controller_spawner,
        move_group,
        rviz,
        gz_bridge,
        camera_perception,
        TimerAction(period=startup_delay, actions=[skill_server]),
    ])
