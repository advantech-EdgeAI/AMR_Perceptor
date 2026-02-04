#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

class DownsampleNode(Node):
    def __init__(self):
        super().__init__('pc_downsampler')

        # Parameters (behavior only)
        self.declare_parameter('keep_ratio', 0.1)
        self.keep_ratio = float(self.get_parameter('keep_ratio').value)

        # Fixed topic names (remap-friendly)
        self.sub = self.create_subscription(
            PointCloud2,
            'points_in',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            'points_out',
            10
        )

        self.get_logger().info(
            f"DownsampleNode started\n"
            f"  keep_ratio: {self.keep_ratio}"
        )

    def callback(self, msg):
        points = np.array(
            list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        )

        if points.size == 0:
            return

        sample_size = int(len(points) * self.keep_ratio)
        if sample_size <= 0:
            self.get_logger().warn('Sample size <= 0, skipping frame')
            return

        sampled = points[
            np.random.choice(len(points), sample_size, replace=False)
        ]

        header = Header(
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id
        )

        out_msg = pc2.create_cloud_xyz32(header, sampled.tolist())
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DownsampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
