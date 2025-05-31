#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class WebcamAndCmdVel(Node):
    def __init__(self):
        super().__init__('webcam_and_cmdvel_node')

        # Görüntü yayıncısı
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Webcam not accessible')

        # cmd_vel dinleyici
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to /cmd_vel')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(msg)
        else:
            self.get_logger().warn('Frame not captured')

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f"[cmd_vel] Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamAndCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
