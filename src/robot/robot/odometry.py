import math

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from messages.msg import Encoder  # noqa: F401
from nav_msgs.msg import Odometry as Odom
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from src.robot.robot.dataclasses.robot import Robot
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from .dataclasses import Wheels, Wheel


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.get_logger().info("Initializing Odometry")

        # Pose
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.get_logger().info("Creating subscription on /encoder topic")
        self.subscription_sub: Subscription = self.create_subscription(Encoder, '/encoder', self.calculate_odom, 10)

        self.get_logger().info("Creating publisher on /odom topic")
        self.odom_pub: Publisher = self.create_publisher(Odom, 'odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot details
        self.get_logger().info("Getting robot details")
        self.robot = Robot()

        # TODO: remove, it's for testing only
        self.calculate_odom(None)

    def calculate_odom(self, encoder: Encoder):

        # Update pulses per wheel
        self.wheels.front_left.update_pulses(encoder.front_left)
        self.wheels.front_right.update_pulses(encoder.front_right)
        self.wheels.rear_left.update_pulses(encoder.rear_left)
        self.wheels.rear_right.update_pulses(encoder.rear_right)

        # Calculate velocities
        vel_x = self.wheels.get_forward_velocity(self.wheel_diameter)
        vel_y = self.wheels.get_right_velocity(self.wheel_diameter)
        vel_yaw = self.wheels.get_angular_velocity(self.wheel_base, self.track_width, self.wheel_diameter)

        # Multiply velocities to match real world
        vel_x *= -1.26
        vel_y *= 1.26
        vel_yaw *= -2.1

        # Update X and Y coordinates
        self.x += vel_x * math.cos(self.yaw) - vel_y * math.sin(self.yaw)
        self.y += vel_y * math.cos(self.yaw) + vel_x * math.sin(self.yaw)
        self.yaw += vel_yaw

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Publish results
        self.publish_odom(vel_x, vel_y, vel_yaw)
        self.publish_tf()


    def publish_odom(self, vel_x, vel_y, vel_yaw):
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom = Odom()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.angular.z = vel_yaw

    def publish_tf(self):
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = Odometry()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()