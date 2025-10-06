import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from math import sin, cos, pi


class OdometryPublisher(Node):
    def __init__(self, L=0.20, W=0.21, R=0.03, pulses_per_rev=396):
        super().__init__('odometry_publisher')

        self.L = L
        self.W = W
        self.R = R
        self.pulses_per_rev = pulses_per_rev

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last tick counts for delta computation
        self.prev_ticks = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}

        # ROS I/O
        self.create_subscription(Int32MultiArray, 'wheel_ticks', self.ticks_cb, 20)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Use time synchronized with ROS
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

    def ticks_cb(self, msg: Int32MultiArray):
        now_time = self.get_clock().now()
        now_ros = now_time.nanoseconds * 1e-9
        dt = now_ros - self.last_time
        if dt <= 0:
            return
        self.last_time = now_ros

        if len(msg.data) < 4:
            return

        ticks = {
            "FL": int(msg.data[0]),
            "FR": int(msg.data[1]),
            "RL": int(msg.data[2]),
            "RR": int(msg.data[3]),
        }

        # Delta pulses since last message
        d = {}
        for wheel in ["FL", "FR", "RL", "RR"]:
            pulses = ticks[wheel] - self.prev_ticks[wheel]
            self.prev_ticks[wheel] = ticks[wheel]
            revs = pulses / self.pulses_per_rev
            d[wheel] = revs * 2 * pi * self.R

        # Mecanum forward kinematics from encoder distances
        vx = ((d["FL"] + d["FR"] + d["RL"] + d["RR"]) / 4.0) / dt
        vy = ((-d["FL"] + d["FR"] + d["RL"] - d["RR"]) / 4.0) / dt
        wz = (-d["FL"] + d["FR"] - d["RL"] + d["RR"]) / (4.0 * (self.L + self.W)) / dt

        # Preserve the same scale adjustments as original node
        vx = vx * -1.26
        vy = vy * 1.26
        wz = wz * -2.1

        # Integrate pose
        self.x += vx * cos(self.theta) * dt - vy * sin(self.theta) * dt
        self.y += vx * sin(self.theta) * dt + vy * cos(self.theta) * dt
        self.theta += wz * dt
        while self.theta > pi:
            self.theta -= 2 * pi
        while self.theta < -pi:
            self.theta += 2 * pi

        # Publish odometry
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

        # Publish TF
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
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


