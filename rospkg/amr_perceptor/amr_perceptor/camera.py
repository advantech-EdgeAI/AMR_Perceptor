#!/usr/bin/env python3

# import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

import sys

class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_node')
        self.declare_parameter('camera_prefix', '/camera')
        self.declare_parameter('video_path', '')
        self.declare_parameter('video_device', 0)
        # output_topic = f"{self.get_parameter('camera_prefix').value}/color/image_raw"
        output_topic = "color/image_raw"
        self.publisher_ = self.create_publisher(Image, output_topic, 1)
        if self.get_parameter('video_path').value:
            self.video_cap = cv2.VideoCapture(self.get_parameter('video_path').value)
        else:
            self.video_cap = cv2.VideoCapture(self.get_parameter('video_device').value)
        if not self.video_cap.isOpened():
            self.get_logger().error("Failed to open video capture device")
            rclpy.shutdown()
            sys.exit(1) 
            

        timer_period = 1/self.video_cap.get(cv2.CAP_PROP_FPS)  # seconds
        # timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.count = 0
        
        ret, cv2_im = self.video_cap.read()
        if ret:
            self.get_logger().info('Camera Node is ready.')

       
    def timer_callback(self):
        
        ret, cv2_im = self.video_cap.read()
        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
            # self.get_logger().info('Publishing an image')
            
def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--video-path", type=str, required=False, default="")
    # parser.add_argument("--video-device", type=int, required=False, default=0)
    # args = parser.parse_args()

    rclpy.init()
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 run amr_remembr camera_node -- --video-device 4