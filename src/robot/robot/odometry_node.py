from math import sin, cos, pi

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from .dataclasses.robot import Robot


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # Get robot details
        self.robot = Robot()

        # Robot position
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info('Creating subscriber on /wheel_speeds topic')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.wheel_speeds_callback
        )

        self.get_logger().info('Creating publisher on /odom topic')
        self.odom_pub = self.create_publisher(Odometry, 'odom')

        self.get_logger().info('Creating TF tree broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Save time for deltatime calculations
        self.last_time = self.get_clock().now().nanoseconds * 1e-3

    def wheel_speeds_callback(self, msg: Float32MultiArray) -> None:
        """
        Updates the position of the robot based on the current velocity
        :param msg: velocities of each wheel
        """

        # Ignore if less than 4 motors are in the array
        if len(msg.data) < 4:
            return

        # Save time for delta time calculations
        now_time = self.get_clock().now()
        now_ros = now_time.nanoseconds * 1e-3
        dt = max(now_ros - self.last_time, 1e-3) #maybe 
        self.last_time = now_ros

        # Calculate robot movement vector from all motor velocities
        vel_front_left, vel_front_right, vel_rear_left, vel_rear_right = msg.data[:4]
        vel_x = ((vel_front_left + vel_front_right + vel_rear_left + vel_rear_right) / 4.0)
        vel_y = ((-vel_front_left + vel_front_right + vel_rear_left - vel_rear_right) / 4.0)
        vel_yaw = (-vel_front_left + vel_front_right - vel_rear_left + vel_rear_right) / (
                    4.0 * (self.robot.robot_length + self.robot.robot_width) / 10000)

        # Magic numbers
        vel_x = vel_x * -1.26
        vel_y = vel_y * 1.26
        vel_yaw = vel_yaw * -2.1

        # Update robot position based on velocity
        self.x += vel_x * cos(self.yaw) * dt - vel_y * sin(self.yaw) * dt
        self.y += vel_x * sin(self.yaw) * dt + vel_y * cos(self.yaw) * dt
        self.yaw += vel_yaw * dt

        # Normalize yaw
        while self.yaw > pi:
            self.yaw -= 2 * pi
        while self.yaw < -pi:
            self.yaw += 2 * pi

        # Publish odometry
        odom = Odometry()

        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.angular.z = vel_yaw

        self.odom_pub.publish(odom)

        # Publish TF tree
        t = TransformStamped()

        t.header.stamp = now_time.to_msg()
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
