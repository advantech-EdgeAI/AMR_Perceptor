#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageResizeNode(Node):
    def __init__(self):
        super().__init__('image_resize')

        # Parameters (only behavior-related)
        self.declare_parameter('scale', 0.5)
        self.declare_parameter('interpolation', 'linear')

        self.scale = float(self.get_parameter('scale').value)
        interp_name = self.get_parameter('interpolation').value.lower()

        interp_modes = {
            'nearest': cv2.INTER_NEAREST,
            'linear': cv2.INTER_LINEAR,
            'area': cv2.INTER_AREA,
            'cubic': cv2.INTER_CUBIC
        }
        self.interp = interp_modes.get(interp_name, cv2.INTER_LINEAR)

        self.bridge = CvBridge()

        # 🔑 FIXED topic names (remap-friendly)
        self.sub = self.create_subscription(
            Image,
            'image_in',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Image,
            'image_out',
            10
        )

        self.get_logger().info(
            f"ImageResizeNode started\n"
            f"  scale: {self.scale}\n"
            f"  interpolation: {interp_name}"
        )

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized = cv2.resize(
                cv_img,
                (0, 0),
                fx=self.scale,
                fy=self.scale,
                interpolation=self.interp
            )

            out_msg = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Resize error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageResizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
