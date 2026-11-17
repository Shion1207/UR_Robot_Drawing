#!/usr/bin/env python3
"""Webcam -> DeepFace emotion only -> stable label and annotated ROS image."""
import signal
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy,QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from emotion_logic import EmotionFilter, classify_result
from camera_geometry import inference_frame, display_box


class EmotionRecognitionNode(Node):
    def __init__(self):
        super().__init__('emotion_recognition_node')
        defaults=dict(camera_index=0,width=640,height=480,inference_width=320,fps=5.,confidence=.60,
                      hold_seconds=.6,show_window=True,mirror=True,max_inference_age=.9)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.options={k:self.get_parameter(k).value for k in defaults}
        cv2.setNumThreads(2)
        self.filter=EmotionFilter(self.options['hold_seconds'],self.options['confidence'])
        self.pub=self.create_publisher(String,'/detected_emotion',1)
        self.image_pub=self.create_publisher(Image,'/emotion/annotated_image',1)
        self.status='WAIT_FACE'
        self.create_subscription(String,'/emotion/status',lambda m:setattr(self,'status',m.data),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_label=''
        self.capture=None
        from deepface import DeepFace
        self.analyze=DeepFace.analyze
        # Warm only the small Emotion network, not identity/age/gender models.
        DeepFace.build_model('Emotion',task='facial_attribute')
        self.capture=cv2.VideoCapture(int(self.options['camera_index']),cv2.CAP_V4L2)
        if not self.capture.isOpened():raise RuntimeError('Cannot open webcam; check /dev/video* and camera_index')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.options['width'])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.options['height'])
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.create_timer(1/max(1.,float(self.options['fps'])),self.on_frame)
        self.get_logger().info('Webcam ready. Show expression -> robot motion. Neutral/no face -> stop.')

    def on_frame(self):
        ok,frame=self.capture.read()
        if not ok:
            self.filter.reset();self.pub.publish(String(data='none'))
            self.get_logger().error('No camera frame; robot must stop.');return
        if self.options['mirror']:frame=cv2.flip(frame,1)
        analysis_frame,scale=inference_frame(frame,int(self.options['inference_width']))
        started=time.monotonic();results=[];error=''
        try:
            results=self.analyze(analysis_frame,actions=['emotion'],detector_backend='opencv',
                                 enforce_detection=True,align=True,silent=True)
        except ValueError:
            error='No face / invalid frame'
        except Exception as exc:
            error='Analysis error: '+str(exc)[:90]
            self.get_logger().error(error,throttle_duration_sec=5.)
        label,score,faces=classify_result(results)
        elapsed=time.monotonic()-started
        if elapsed>self.options['max_inference_age']:
            label,score='none',0.;error='Inference too slow: robot stopped'
        stable=self.filter.update(label,score,faces,time.monotonic())
        self.pub.publish(String(data=stable))
        if stable!=self.last_label:
            self.get_logger().info(f'Expression={label} score={score:.2f} published={stable}')
            self.last_label=stable
        for result in results:
            x,y,w,h=display_box(result.get('region',{}),scale)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,220,0),2)
        lines=[f'AI: {label} score={score:.2f} faces={faces} inference={elapsed:.2f}s',
               'Published: '+stable,self.status[:85],
               'Happy: left | Sad: right | Angry: back | Fear: forward',
               'Surprise: one turn | Neutral: STOP | Q: exit',error]
        # Extra panel keeps text out of the face region. No recording is stored.
        output=cv2.copyMakeBorder(frame,145,0,0,0,cv2.BORDER_CONSTANT,value=(28,28,28))
        for i,line in enumerate(lines):cv2.putText(output,line,(8,22+i*23),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
        message=Image();message.header.stamp=self.get_clock().now().to_msg()
        message.height,message.width=output.shape[:2];message.encoding='bgr8';message.step=message.width*3
        message.data=output.tobytes();self.image_pub.publish(message)
        if self.options['show_window']:
            try:
                cv2.imshow('Assignment 8 - Facial Expressions',output)
                key=cv2.waitKey(1)&0xff
                if key in (27,ord('q')) or cv2.getWindowProperty('Assignment 8 - Facial Expressions',cv2.WND_PROP_VISIBLE)<1:
                    self.pub.publish(String(data='none'));raise KeyboardInterrupt
            except cv2.error as exc:
                self.options['show_window']=False
                self.get_logger().warning('GUI unavailable. Use rqt_image_view /emotion/annotated_image. '+str(exc)[:80])

    def destroy_node(self):
        if rclpy.ok():self.pub.publish(String(data='none'))
        if self.capture:self.capture.release()
        try:cv2.destroyAllWindows()
        except cv2.error:pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=EmotionRecognitionNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
