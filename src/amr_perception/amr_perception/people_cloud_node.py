"""Turn 3D person detections into a point cloud that Nav2's costmap can use.

Subscribes:  /detections_3d  (vision_msgs/Detection3DArray, map frame)
Publishes:   /people_cloud   (sensor_msgs/PointCloud2, map frame)

Each detected person becomes a small disc of points, so the costmap marks
a patch of cells around them as an obstacle.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray


class PeopleCloud(Node):

    def __init__(self):
        super().__init__('people_cloud')
        self.declare_parameter('radius', 0.35)     # metres: size of the disc around a person
        self.declare_parameter('spacing', 0.05)    # metres between points (= costmap resolution)
        self.declare_parameter('height', 0.5)      # metres above ground
        # YOLO labels to treat as people. A person seen from very close is
        # sometimes labelled "tie" or "umbrella"; those are ignored.
        self.declare_parameter('classes', ['person'])
        self.classes = set(self.get_parameter('classes').value)
        self.radius = self.get_parameter('radius').value
        self.spacing = self.get_parameter('spacing').value
        self.height = self.get_parameter('height').value

        self.create_subscription(Detection3DArray, '/detections_3d', self.on_detections, 10)
        self.pub = self.create_publisher(PointCloud2, '/people_cloud', 10)
        self.get_logger().info('Publishing person discs on /people_cloud')

    def on_detections(self, msg):
        points = []
        n = int(self.radius / self.spacing)
        for det in msg.detections:
            label = det.results[0].hypothesis.class_id if det.results else ''
            if label not in self.classes:
                continue
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            # fill a disc: every grid point within `radius` of the person
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    dx, dy = i * self.spacing, j * self.spacing
                    if math.hypot(dx, dy) <= self.radius:
                        points.append((cx + dx, cy + dy, self.height))
        # publish even when empty, so the costmap knows nobody is there
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, points))


def main(args=None):
    rclpy.init(args=args)
    node = PeopleCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
