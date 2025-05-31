#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import subprocess

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Subscribed to /image_raw')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Webcam View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def start_teleop():
    subprocess.run([
        'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
    ])

def main():
    # ROS 2 init
    rclpy.init()
    
    # Teleop in separate thread
    teleop_thread = threading.Thread(target=start_teleop, daemon=True)
    teleop_thread.start()

    # Image display
    image_viewer = ImageViewer()
    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        image_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
