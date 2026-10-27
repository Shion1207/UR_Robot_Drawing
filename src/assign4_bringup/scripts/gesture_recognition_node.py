#!/usr/bin/env python3
"""Recognize six hand gestures from a real laptop webcam, never Gazebo RGB."""
import os
import signal
from pathlib import Path
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

from gesture_logic import GestureFilter, LABELS, WaveDetector

CONNECTIONS = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
               (5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),
               (15,16),(13,17),(0,17),(17,18),(18,19),(19,20)]


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        model = Path(get_package_share_directory('assign4_bringup')) / 'models/gesture_recognizer.task'
        for key, value in dict(camera_index=0, model_path=str(model), confidence=0.65,
                               hold_seconds=0.25, fps=10.0, show_window=True,
                               mirror=True, width=640, height=480).items():
            self.declare_parameter(key, value)
        self.publisher = self.create_publisher(String, '/detected_gesture', 1)
        self.image_publisher = self.create_publisher(Image, '/gesture/annotated_image', 1)
        self.bridge = CvBridge()
        self.behavior_status = 'IDLE: choose gesture, then Thumb_Up'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/gesture_behavior/status', self.on_status, qos)
        self.filter = GestureFilter(float(self.get_parameter('hold_seconds').value))
        self.wave = WaveDetector()
        self.confidence = float(self.get_parameter('confidence').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.mirror = bool(self.get_parameter('mirror').value)
        self.capture = self.recognizer = None
        self.last_timestamp = -1
        self.last_label = None
        self.failures = 0
        model_path = str(Path(str(self.get_parameter('model_path').value)).expanduser())
        if not Path(model_path).is_file():
            raise RuntimeError('Thiếu gesture_recognizer.task. Chạy ./scripts/install_gesture_deps.bash')
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError('Thiếu MediaPipe. Chạy ./scripts/install_gesture_deps.bash rồi run_gestures.bash') from exc
        self.mp = mp
        # Two hands are detected so ambiguous multi-person input can be rejected.
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=2, min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5, min_tracking_confidence=0.5)
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        index = int(self.get_parameter('camera_index').value)
        self.capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.recognizer.close()
            raise RuntimeError(f'Không mở được /dev/video{index}. Kiểm tra webcam/quyền video; thử camera_index:=2.')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('width').value))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('height').value))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fps = float(self.get_parameter('fps').value)
        if fps <= 0:
            raise ValueError('fps must be positive')
        # Do not call imshow in a headless build: it throws and used to kill YOLO.
        if self.show_window and ('GUI:                           NONE' in cv2.getBuildInformation()
                                 or not os.environ.get('DISPLAY')):
            self.show_window = False
            self.get_logger().warn('Không có OpenCV GUI. Xem /gesture/annotated_image bằng rqt_image_view.')
        self.timer = self.create_timer(1.0/fps, self.on_frame)
        self.get_logger().info(f'Webcam {index} + MediaPipe sẵn sàng. Đưa một bàn tay vào khung hình.')

    def on_status(self, msg):
        self.behavior_status = msg.data

    def on_frame(self):
        ok, frame = self.capture.read()
        now = time.monotonic()
        if not ok:
            self.publisher.publish(String(data='None'))
            self.failures += 1
            self.get_logger().warn('Mất ảnh webcam: robot sẽ dừng.', throttle_duration_sec=3.0)
            if self.failures >= 30:
                raise RuntimeError('Webcam không trả ảnh; kiểm tra thiết bị và chạy lại run_gestures.bash.')
            return
        self.failures = 0
        if self.mirror:
            frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp = max(self.last_timestamp+1, int(now*1000))
        self.last_timestamp = timestamp
        result = self.recognizer.recognize_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), timestamp)
        raw, score, wrist = 'None', 0.0, None
        if len(result.hand_landmarks) == 1 and result.gestures and result.gestures[0]:
            top = result.gestures[0][0]
            score = float(top.score)
            if top.category_name in LABELS and score >= self.confidence:
                raw = top.category_name
            wrist = result.hand_landmarks[0][0].x
        is_wave = self.wave.update(raw, wrist, now)
        if is_wave:
            raw = 'Wave'
        # During a transient classification change we publish None rather than
        # inventing a continued hand-control heartbeat.
        label = self.filter.update(raw, now)
        self.publisher.publish(String(data=label))
        if label != self.last_label:
            self.get_logger().info(f'Cử chỉ: {label}; confidence={score:.2f}')
            self.last_label = label
        self.draw(frame, result.hand_landmarks, raw, label, score)
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'laptop_webcam'
        self.image_publisher.publish(image)
        if self.show_window:
            try:
                cv2.imshow('Assignment 6 - Hand Gestures', frame)
                cv2.waitKey(1)  # Observation only; keyboard does not command the robot.
            except cv2.error:
                self.show_window = False
                self.get_logger().warn('GUI OpenCV không dùng được. Mở rqt_image_view /gesture/annotated_image.')

    def draw(self, frame, hands, raw, label, score):
        height, width = frame.shape[:2]
        for hand in hands:
            points = [(int(p.x*width), int(p.y*height)) for p in hand]
            for a, b in CONNECTIONS:
                cv2.line(frame, points[a], points[b], (0, 220, 0), 2)
            for p in points:
                cv2.circle(frame, p, 4, (0, 120, 255), -1)
        lines = [f'Gesture: {raw}  confidence: {score:.2f}', f'Published: {label}',
                 self.behavior_status, 'Point: Fwd/Back | Victory: Circle | Wave: Greet',
                 'Thumb Up: CONFIRM | Palm: STOP | Fist: CANCEL',
                 'Keep one hand visible while moving. No hand = STOP.']
        cv2.rectangle(frame, (0, 0), (width, 150), (30, 30, 30), -1)
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (8, 22+i*24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.46, (255, 255, 255), 1, cv2.LINE_AA)

    def destroy_node(self):
        if rclpy.ok():
            self.publisher.publish(String(data='None'))
        if self.capture is not None:
            self.capture.release()
        if self.recognizer is not None:
            self.recognizer.close()
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognitionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
