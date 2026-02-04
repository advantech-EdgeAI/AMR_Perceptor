import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class ResizeCamera(Node):
    def __init__(self):
        super().__init__('resize_camera')

        # Parameters
        self.declare_parameter('camera_prefix', 'camera')
        self.declare_parameter('ratio', 0.5)

        self.prefix = self.get_parameter('camera_prefix').get_parameter_value().string_value
        self.ratio = float(self.get_parameter('ratio').get_parameter_value().double_value)

        self.bridge = CvBridge()

        # Topics
        color_image = f'/{self.prefix}/color/image_raw'
        depth_image = f'/{self.prefix}/depth/image_raw'
        color_info = f'/{self.prefix}/color/camera_info'
        depth_info = f'/{self.prefix}/depth/camera_info'

        color_image_out = f'/{self.prefix}/color_resized/image_raw'
        depth_image_out = f'/{self.prefix}/depth_resized/image_raw'
        color_info_out = f'/{self.prefix}/color_resized/camera_info'
        depth_info_out = f'/{self.prefix}/depth_resized/camera_info'

        # Subscribers
        self.create_subscription(Image, color_image, self.color_cb, 10)
        self.create_subscription(Image, depth_image, self.depth_cb, 10)
        self.create_subscription(CameraInfo, color_info, self.color_info_cb, 10)
        self.create_subscription(CameraInfo, depth_info, self.depth_info_cb, 10)

        # Publishers
        self.color_pub = self.create_publisher(Image, color_image_out, 10)
        self.depth_pub = self.create_publisher(Image, depth_image_out, 10)
        self.color_info_pub = self.create_publisher(CameraInfo, color_info_out, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, depth_info_out, 10)

        self.get_logger().info(f"Resizing {self.prefix} with ratio={self.ratio}")

    # Resize CameraInfo correctly
    def resize_camera_info(self, info):
        new_info = CameraInfo()
        new_info = info

        new_info.width = int(info.width * self.ratio)
        new_info.height = int(info.height * self.ratio)

        new_info.k[0] *= self.ratio  # fx
        new_info.k[2] *= self.ratio  # cx
        new_info.k[4] *= self.ratio  # fy
        new_info.k[5] *= self.ratio  # cy

        new_info.p[0] *= self.ratio  # fx
        new_info.p[2] *= self.ratio  # cx
        new_info.p[5] *= self.ratio  # fy
        new_info.p[6] *= self.ratio  # cy

        return new_info

    def color_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        resized = cv2.resize(img, None, fx=self.ratio, fy=self.ratio)
        out = self.bridge.cv2_to_imgmsg(resized, 'bgr8')
        out.header = msg.header
        self.color_pub.publish(out)

    def depth_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        resized = cv2.resize(img, None, fx=self.ratio, fy=self.ratio, interpolation=cv2.INTER_NEAREST)
        out = self.bridge.cv2_to_imgmsg(resized, encoding='passthrough')
        out.header = msg.header
        self.depth_pub.publish(out)

    def color_info_cb(self, msg):
        self.color_info_pub.publish(self.resize_camera_info(msg))

    def depth_info_cb(self, msg):
        self.depth_info_pub.publish(self.resize_camera_info(msg))

def main():
    rclpy.init()
    node = ResizeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
