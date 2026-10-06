#!/usr/bin/env python3
"""Nhan dien 5 cube tu camera RGB tren cao va publish pose trong frame world."""

import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from llm_planner.perception import (
    CameraModel,
    detect_colored_objects,
    pixel_to_world,
    scene_from_detections,
)


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__("camera_perception")
        self.object_names: List[str] = [
            n for n in self.declare_parameter("object_names", [""]).value if n
        ]
        self.zone_names: List[str] = [
            n for n in self.declare_parameter("zone_names", [""]).value if n
        ]
        if not self.object_names or not self.zone_names:
            raise RuntimeError("camera_perception chua duoc nap config/scene.yaml")

        self.object_colors = {
            name: self.declare_parameter(f"objects.{name}.color", "").value.lower()
            for name in self.object_names
        }
        missing = [name for name, color in self.object_colors.items() if not color]
        if missing:
            raise RuntimeError(f"Cac object chua khai bao color: {missing}")
        self.zones = {
            name: tuple(self.declare_parameter(f"zones.{name}.position", [0.0, 0.0, 0.0]).value)
            for name in self.zone_names
        }

        self.camera_position = tuple(
            float(v) for v in self.declare_parameter("camera.position", [0.15, 0.0, 1.20]).value
        )
        self.cube_size = float(self.declare_parameter("cube_size", 0.04).value)
        self.zone_size = float(self.declare_parameter("zone_size", 0.08).value)
        self.min_area = int(self.declare_parameter("perception.min_blob_area", 120).value)
        self.max_area = int(self.declare_parameter("perception.max_blob_area", 1800).value)
        self.smoothing = float(self.declare_parameter("perception.smoothing", 0.35).value)
        self.image_topic = self.declare_parameter(
            "camera.image_topic", "/overhead_camera/image").value
        self.info_topic = self.declare_parameter(
            "camera.info_topic", "/overhead_camera/camera_info").value

        self.camera_model: Optional[CameraModel] = None
        self.positions: Dict[str, Tuple[float, float, float]] = {}
        self.frame_count = 0

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pose_pub = self.create_publisher(PoseArray, "camera/object_poses", latched)
        self.state_pub = self.create_publisher(String, "camera/scene_state", latched)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"Camera perception dang cho anh: {self.image_topic} | objects: {self.object_names}")

    def _on_info(self, msg: CameraInfo):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.camera_model = CameraModel(
            fx=float(msg.k[0]), fy=float(msg.k[4]), cx=float(msg.k[2]), cy=float(msg.k[5]),
            position=self.camera_position,
        )

    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray:
        channels = 4 if msg.encoding.lower() in ("rgba8", "bgra8") else 3
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = row[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            return image[:, :, ::-1].copy()
        if encoding == "bgr8":
            return image.copy()
        if encoding == "rgba8":
            return image[:, :, [2, 1, 0]].copy()
        if encoding == "bgra8":
            return image[:, :, :3].copy()
        raise ValueError(f"Encoding camera khong ho tro: {msg.encoding}")

    def _on_image(self, msg: Image):
        if self.camera_model is None:
            self.get_logger().warn("Chua nhan CameraInfo", throttle_duration_sec=5.0)
            return
        try:
            bgr = self._to_bgr(msg)
            detections = detect_colored_objects(
                bgr, self.object_colors, self.min_area, self.max_area)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Khong xu ly duoc anh camera: {exc}")
            return

        visible = set(detections)
        top_z = self.cube_size
        center_z = self.cube_size / 2.0
        for name, (u, v, _) in detections.items():
            x, y, _ = pixel_to_world(u, v, self.camera_model, top_z)
            measured = (x, y, center_z)
            old = self.positions.get(name)
            if old is None:
                self.positions[name] = measured
            else:
                a = self.smoothing
                self.positions[name] = tuple(a * n + (1.0 - a) * o
                                             for n, o in zip(measured, old))

        self.frame_count += 1
        poses = PoseArray()
        poses.header = msg.header
        poses.header.frame_id = "world"
        for name in self.object_names:
            pose = Pose()
            if name in visible and name in self.positions:
                pose.position.x, pose.position.y, pose.position.z = self.positions[name]
                pose.orientation.w = 1.0
            else:
                pose.position.x = pose.position.y = pose.position.z = math.nan
            poses.poses.append(pose)
        self.pose_pub.publish(poses)

        state = scene_from_detections(self.positions, visible, self.zones, self.zone_size)
        state["camera_ready"] = all(name in self.positions for name in self.object_names)
        state["all_visible"] = all(name in visible for name in self.object_names)
        state["sequence"] = self.frame_count
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main():
    rclpy.init()
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
