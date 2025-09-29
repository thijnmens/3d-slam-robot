#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocitySubscriberNode(Node):

    def __init__(self):
        super().__init__("velocity_subscriber")
        self.velocity_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.velocity_callback, 10)

    def velocity_callback(self, msg: Twist):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
