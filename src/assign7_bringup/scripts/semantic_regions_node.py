#!/usr/bin/env python3
"""Publish colored safe-region cells, labelled goals and source-map checked metadata."""
from pathlib import Path
import signal

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from semantic_map import load_regions

COLORS = [(0.25,0.57,0.88), (0.96,0.67,0.25), (0.41,0.75,0.49)]


class SemanticRegions(Node):
    def __init__(self):
        super().__init__('semantic_regions_node')
        share = Path(get_package_share_directory('assign7_bringup'))
        self.declare_parameter('regions_file', str(share/'semantic/regions.json'))
        self.declare_parameter('map_yaml', str(share/'maps/map.yaml'))
        data, self.occupancy = load_regions(self.get_parameter('regions_file').value,
                                           self.get_parameter('map_yaml').value)
        self.data = data
        self.labels = cv2.imread(str(Path(self.get_parameter('regions_file').value).parent/'region_labels.png'), 0)
        if self.labels is None or self.labels.shape != self.occupancy.image.shape:
            raise ValueError('Thiếu hoặc sai kích thước region_labels.png; chạy segment_map.bash')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/semantic_regions', qos)
        self.publish()
        self.create_timer(5.0, self.publish)
        self.get_logger().info('Đã publish 3 vùng có nhãn và điểm đích an toàn lên /semantic_regions.')

    def publish(self):
        array = MarkerArray()
        for i, region in enumerate(self.data['regions']):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id, marker.type = 'region_cells', i, Marker.CUBE_LIST
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = self.occupancy.resolution*2
            marker.scale.z = 0.015
            marker.color.r, marker.color.g, marker.color.b = COLORS[i]
            marker.color.a = 0.30
            for v, u in np.argwhere(self.labels[::2, ::2] == i+1):
                x, y = self.occupancy.pixel_to_world(int(u)*2, int(v)*2)
                marker.points.append(Point(x=x, y=y, z=0.025))
            array.markers.append(marker)
            text = Marker()
            text.header.frame_id = 'map'
            text.ns, text.id, text.type = 'region_labels', i, Marker.TEXT_VIEW_FACING
            text.pose.orientation.w = 1.0
            text.pose.position.x, text.pose.position.y = map(float, region['goal'][:2])
            text.pose.position.z = 0.55
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            # RViz Ogre default font is not guaranteed to contain Vietnamese glyphs.
            text.text = f"{region['id']} - "+['Phong hoc','Phong thi nghiem','Hanh lang'][i]
            array.markers.append(text)
            goal = Marker()
            goal.header.frame_id = 'map'
            goal.ns, goal.id, goal.type = 'region_goals', i, Marker.SPHERE
            goal.pose.position.x, goal.pose.position.y = map(float, region['goal'][:2])
            goal.pose.position.z, goal.pose.orientation.w = 0.12, 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.20
            goal.color.r, goal.color.g, goal.color.b = COLORS[i]
            goal.color.a = 1.0
            array.markers.append(goal)
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticRegions()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
