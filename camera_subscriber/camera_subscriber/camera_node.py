#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.window_name = "camera_interface"

        self.image_width = 700
        self.image_height = 512

        # SUB: obraz z kamery
        self.create_subscription(Image, "/image_raw", self.image_callback, 10)

        # PUB: punkt kliknięcia
        self.point_pub = self.create_publisher(Point, "/point", 10)

        # SUB: sterowanie robotem
        self.create_subscription(Point, "/point", self.point_callback, 10)

        # PUB: wysyłanie komend
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # -------------------- OBSŁUGA KAMERY + KLIKÓW --------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.image_height, self.image_width = frame.shape[:2]

        cv2.imshow(self.window_name, frame)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # RESET poprzedniego kierunku
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            self.get_logger().info("[RESET] Stopping robot before new direction")

            # Publikacja punktu kliknięcia
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.0
            self.point_pub.publish(msg)

            self.get_logger().info(f"[CLICK] New point published: ({x}, {y})")

    # -------------------- STEROWANIE ROBOTEM --------------------
    def point_callback(self, msg):
        cy = self.image_height / 2

        twist = Twist()

        if msg.y < cy:
            twist.linear.x = 0.5   # do przodu
            self.get_logger().info("[MOVE] Point above center → FORWARD")
        else:
            twist.linear.x = -0.5  # do tyłu
            self.get_logger().info("[MOVE] Point below center → BACKWARD")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

