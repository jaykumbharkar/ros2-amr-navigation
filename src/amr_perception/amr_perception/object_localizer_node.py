"""Turn 2D YOLO detections into 3D positions using the depth camera."""
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import message_filters
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray



class ObjectLocalizer(Node):

    def __init__(self):
        super().__init__('object_localizer')
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None    # filled from camera_info

        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_info, 10)

        # Pair each detection with the depth image from the SAME moment.
        # RELIABLE: a depth image is 1.2 MB; BEST_EFFORT loses whole images.
        depth_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/detections')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=depth_qos)
        sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=20, slop=0.05)
        sync.registerCallback(self.on_detections)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.det_pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detections_3d/markers', 10)
        self.get_logger().info(f'Localizing detections in the "{self.target_frame}" frame')

    def on_info(self, msg):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def on_detections(self, det_msg, depth_msg):
        if self.fx is None:
            return
        depth = self.bridge.imgmsg_to_cv2(depth_msg)            # 2D array of metres
        stamp = Time.from_msg(det_msg.header.stamp)
        try:
            to_target = self.tf_buffer.lookup_transform(
                self.target_frame, det_msg.header.frame_id, stamp)
            to_robot = self.tf_buffer.lookup_transform(
                'base_footprint', det_msg.header.frame_id, stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No transform yet: {e}', throttle_duration_sec=2.0)
            return

        out = Detection3DArray()
        out.header.stamp = det_msg.header.stamp
        out.header.frame_id = self.target_frame
        markers = MarkerArray()

        for i, det in enumerate(det_msg.detections):
            u, v = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y

            # Distance: median depth of the box's middle third
            patch = depth[int(v - h / 6):int(v + h / 6) + 1, int(u - w / 6):int(u + w / 6) + 1]
            valid = patch[np.isfinite(patch)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))

            # Pinhole model: pixel (u, v) at depth z -> 3D point in camera optical frame
            p = PointStamped()
            p.header = det_msg.header
            p.point.x = (u - self.cx) * z / self.fx
            p.point.y = (v - self.cy) * z / self.fy
            p.point.z = z

            p_target = do_transform_point(p, to_target)          # e.g. map frame
            p_robot = do_transform_point(p, to_robot)            # robot frame
            label = det.results[0].hypothesis.class_id if det.results else 'object'
            self.get_logger().info(
                f'{label}: {p_robot.point.x:.2f} m ahead, {p_robot.point.y:+.2f} m left '
                f'-> {self.target_frame} ({p_target.point.x:.2f}, {p_target.point.y:.2f})',
                throttle_duration_sec=1.0)

            d3 = Detection3D()
            d3.header = out.header
            d3.results = det.results
            d3.bbox.center.position = p_target.point
            d3.bbox.size.x = d3.bbox.size.y = w * z / self.fx     # box width in metres
            d3.bbox.size.z = h * z / self.fy                     # box height in metres
            out.detections.append(d3)
            markers.markers += self.make_markers(i, label, p_target, out.header)

        self.det_pub.publish(out)
        self.marker_pub.publish(markers)


    def make_markers(self, i, label, p, header):
        sphere = Marker()
        sphere.header = header
        sphere.ns, sphere.id, sphere.type = 'objects', i, Marker.SPHERE
        sphere.pose.position = p.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.5, 0.0, 0.9
        sphere.lifetime = Duration(seconds=0.5).to_msg()     # disappears if not seen again

        text = Marker()
        text.header = header
        text.ns, text.id, text.type = 'labels', i, Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y = p.point.x, p.point.y
        text.pose.position.z = p.point.z + 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.3
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        text.lifetime = sphere.lifetime
        return [sphere, text]


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
