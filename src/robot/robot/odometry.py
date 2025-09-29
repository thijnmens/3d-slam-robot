import math

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from messages.msg import Encoder  # noqa: F401
from nav_msgs.msg import Odometry as Odom
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from .dataclasses.robot import Robot
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.get_logger().info("Initializing Odometry")

        # Pose
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        # Save time of last update for delta time calculation
        self.last_time = self.get_clock().now()

        # /encoder subscription
        self.get_logger().info("Creating subscription on /encoder topic")
        self.subscription_sub: Subscription = self.create_subscription(Encoder, '/encoder', self.calculate_odom, 10)

        # /odom publisher
        self.get_logger().info("Creating publisher on /odom topic")
        self.odom_pub: Publisher = self.create_publisher(Odom, 'odom', 10)

        # /tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot details
        self.get_logger().info("Getting robot details")
        self.robot = Robot()

    def calculate_odom(self, encoder: Encoder) -> None:
        """
        Calculate the odometry of the robot based on the new encoder data.
        :param encoder: new encoder data.
        """

        # Compute delta time since last update
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Update pulses per wheel
        self.robot.front_left.pulses = encoder.front_left
        self.robot.front_right.pulses = encoder.front_right
        self.robot.rear_left.pulses = encoder.rear_left
        self.robot.rear_right.pulses = encoder.rear_right

        # Calculate velocities
        vel_x = self.robot.get_forward_velocity()
        vel_y = self.robot.get_right_velocity()
        vel_yaw = self.robot.get_angular_velocity()

        # Update X, Y and Yaw
        self.x += vel_x * math.cos(self.yaw) * dt - vel_y * math.sin(self.yaw) * dt
        self.y += vel_x * math.sin(self.yaw) * dt + vel_y * math.cos(self.yaw) * dt
        self.yaw += vel_yaw * dt

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Publish results
        self.publish_odom(vel_x, vel_y, vel_yaw)
        self.publish_tf()


    def publish_odom(self, vel_x, vel_y, vel_yaw) -> None:
        """
        Publishes the odometry of the robot to /odom
        :param vel_x: Linear velocity on the X axis
        :param vel_y: Linear velocity on the Y axis
        :param vel_yaw: Angular velocity on the Z axis
        """
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

        self.odom_pub.publish(odom)

    def publish_tf(self) -> None:
        """
        Publishes the transformation matrix to /tf
        """

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