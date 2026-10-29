"""YOLO object detector for the robot's camera.

Subscribes:  /camera/image_raw          (sensor_msgs/Image)
Publishes:   /detections                (vision_msgs/Detection2DArray)
             /detections/image          (sensor_msgs/Image, boxes drawn on the picture)
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')

        # Parameters: can be changed at start-up with --ros-args -p name:=value
        self.declare_parameter('model', '~/yolo11n.pt')  # downloaded here if missing
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('device', '')  # '' = GPU if available, 'cpu' to force CPU
        model_path = os.path.expanduser(self.get_parameter('model').value)
        self.confidence = self.get_parameter('confidence').value
        image_topic = self.get_parameter('image_topic').value
        self.device = self.get_parameter('device').value or None
        
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Keep only the newest image (depth=1): if YOLO is slower than the
        # camera, old frames are dropped instead of piling up and adding delay.
        newest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self.on_image, newest_only)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.img_pub = self.create_publisher(Image, '/detections/image', 10)

        self.frames = 0
        self.inference_ms = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'YOLO model {model_path} loaded, listening on {image_topic}')

    def on_image(self, msg):
        # ROS image -> OpenCV image (a NumPy array, BGR colour order)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.perf_counter()
        result = self.model(frame, conf=self.confidence, device=self.device, verbose=False)[0]
        self.inference_ms += (time.perf_counter() - start) * 1000.0
        self.frames += 1

        # One Detection2D per box. The header (time + camera frame) is copied
        # from the image, so later nodes know exactly when and where it was seen.
        detections = Detection2DArray()
        detections.header = msg.header
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[int(box.cls)]
            hypothesis.hypothesis.score = float(box.conf)
            det.results.append(hypothesis)
            detections.detections.append(det)
        self.det_pub.publish(detections)

        # Picture with boxes and labels drawn on it, for viewing in RViz
        annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
        annotated.header = msg.header
        self.img_pub.publish(annotated)

    def report(self):
        if self.frames:
            self.get_logger().info(
                f'{self.frames / 5.0:.1f} FPS, {self.inference_ms / self.frames:.0f} ms per image')
        self.frames = 0
        self.inference_ms = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
