from math import sin, cos, pi

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self, L=0.20, W=0.21):
        super().__init__('odometry')
        self.L = L
        self.W = W
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        self.subscription = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.wheel_speeds_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def wheel_speeds_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        now_time = self.get_clock().now()
        now_ros = now_time.nanoseconds * 1e-9
        dt = max(now_ros - self.last_time, 1e-3)
        self.last_time = now_ros

        v_fl, v_fr, v_rl, v_rr = msg.data[:4]
        vx = ((v_fl + v_fr + v_rl + v_rr) / 4.0)
        vy = ((-v_fl + v_fr + v_rl - v_rr) / 4.0)
        wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.L + self.W))

        vx = vx * -1.26
        vy = vy * 1.26
        wz = wz * -2.1

        self.x += vx * cos(self.theta) * dt - vy * sin(self.theta) * dt
        self.y += vx * sin(self.theta) * dt + vy * cos(self.theta) * dt
        self.theta += wz * dt

        while self.theta > pi:
            self.theta -= 2 * pi
        while self.theta < -pi:
            self.theta += 2 * pi

        odom = Odometry()

        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

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
