#!/usr/bin/env python3
"""Run Ultralytics YOLO on the TurtleBot 4 RGB camera stream."""

import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDetector(Node):
    """Subscribe to OAK-D images, detect objects and publish annotated images."""

    def __init__(self) -> None:
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('annotated_topic', '/yolo/annotated_image')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('inference_interval', 0.20)
        # OpenCV from pip is headless on this machine, so displaying through
        # cv2.imshow() crashes the node. View the published annotated topic in
        # rqt_image_view instead.
        self.declare_parameter('show_window', False)
        self.declare_parameter('publish_annotated', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        annotated_topic = str(self.get_parameter('annotated_topic').value)
        model_path = str(self.get_parameter('model_path').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = str(self.get_parameter('device').value)
        self.inference_interval = float(
            self.get_parameter('inference_interval').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value)

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Chưa cài ultralytics. Chạy ./scripts/install_yolo_deps.bash.') from exc

        self.get_logger().info(f'Đang nạp mô hình YOLO: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as exc:
            raise RuntimeError(
                f'Không thể nạp mô hình {model_path}: {exc}') from exc

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, annotated_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.last_inference = 0.0
        self.last_detection_log = 0.0
        self.last_image_time: Optional[float] = None
        self.watchdog_started = time.monotonic()
        self.create_timer(5.0, self._camera_watchdog)

        self.get_logger().info(
            f'YOLO sẵn sàng, đang chờ ảnh trên {self.image_topic}.')

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        self.last_image_time = now
        if now - self.last_inference < self.inference_interval:
            return
        self.last_inference = now

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(
                f'Không chuyển được ảnh ROS sang OpenCV: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        try:
            result = self.model.predict(
                source=frame,
                conf=self.confidence,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(
                f'Lỗi YOLO inference: {exc}', throttle_duration_sec=5.0)
            return

        annotated = result.plot()
        detections = []
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = str(result.names[class_id])
            detections.append(f'{class_name} {confidence:.2f}')

        if detections and now - self.last_detection_log >= 1.0:
            self.get_logger().info('Phát hiện: ' + ', '.join(detections))
            self.last_detection_log = now

        if self.publish_annotated:
            try:
                output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                output.header = message.header
                self.publisher.publish(output)
            except CvBridgeError as exc:
                self.get_logger().error(
                    f'Không publish được ảnh YOLO: {exc}',
                    throttle_duration_sec=5.0,
                )

        if self.show_window:
            cv2.imshow('TurtleBot 4 - YOLO Detection', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Đã nhấn Q, đóng cửa sổ YOLO.')
                self.show_window = False
                cv2.destroyAllWindows()

    def _camera_watchdog(self) -> None:
        now = time.monotonic()
        reference = self.last_image_time or self.watchdog_started
        if now - reference > 8.0:
            self.get_logger().warn(
                f'Không nhận được ảnh từ {self.image_topic}. '
                'Kiểm tra Gazebo và topic camera.',
                throttle_duration_sec=10.0,
            )

    def destroy_node(self) -> bool:
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

