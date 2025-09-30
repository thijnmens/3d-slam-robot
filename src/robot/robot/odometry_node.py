from math import sin, cos, pi
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from .dataclasses.robot import Robot
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # Robot data
        self.robot = Robot()

        # Robot location
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Delta time
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        # Wheel speed subscriber
        self.get_logger().info("Creating subscription on /wheel_speeds topic")
        self.subscription = self.create_subscription(
            Float32MultiArray, '/wheel_speeds', self.wheel_speeds_callback, 10
        )

        # Odometry publisher
        self.get_logger().info("Creating publisher on /odom topic")
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF tree broadcaster
        self.get_logger().info("Creating TF broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)

    def wheel_speeds_callback(self, msg: Float32MultiArray):
        # Check if all wheels are included
        if len(msg.data) < 4:
            return

        # Save time for delta time calculations
        now_time = self.get_clock().now()
        now_ros = now_time.nanoseconds * 1e-9
        dt = max(now_ros - self.last_time, 1e-3)
        self.last_time = now_ros

        # Calculate velocities
        vel_fl, vel_fr, vel_rl, vel_rr = msg.data[:4]
        vel_x = ((vel_fl + vel_fr + vel_rl + vel_rr) / 4.0)
        vel_y = ((-vel_fl + vel_fr + vel_rl - vel_rr) / 4.0)
        vel_yaw = (-vel_fl + vel_fr - vel_rl + vel_rr) / (4.0 * (self.robot.robot_length + self.robot.robot_width))

        # Magic numbers
        vel_x = vel_x * -1.26
        vel_y = vel_y * 1.26
        vel_yaw = vel_yaw * -2.1

        # Calculate new position
        self.x += vel_x * cos(self.yaw) * dt - vel_y * sin(self.yaw) * dt
        self.y += vel_x * sin(self.yaw) * dt + vel_y * cos(self.yaw) * dt
        self.yaw += vel_yaw * dt

        # Normalize radials
        while self.yaw > pi:
            self.yaw -= 2 * pi
        while self.yaw < -pi:
            self.yaw += 2 * pi

        # Convert to quaternion
        quat = quaternion_from_euler(0.0, 0.0, self.yaw)

        # Publish new position to odometry
        self.publish_odom(now_time.to_msg(), vel_x, vel_y, vel_yaw, quat)

        # Update transform tree
        self.publish_tf(now_time.to_msg(), quat)

    def publish_odom(self, stamp: Time, vel_x: float, vel_y: float, vel_yaw: float, q: Tuple[float, float, float, float]):
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.angular.z = vel_yaw

        self.odom_pub.publish(odom)

    def publish_tf(self, stamp: Time, q: Tuple[float, float, float, float]):
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
