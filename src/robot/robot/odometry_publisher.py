import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from .dataclasses.robot import Robot
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Int64MultiArray
from math import sin, cos, pi


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Get robot data
        self.robot = Robot()

        # Keep track of encoder counts
        self.encoder_counts = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self.prev_initialized = False

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_counts = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self.last_t = self.get_clock().now().nanoseconds * 1e-9

        # Subscribe to encoder counts
        self.get_logger().info("Subscribing to /encoder_counts")
        self.create_subscription(Int64MultiArray, 'encoder_counts', self.on_encoder_counts, 10)

        # Publish to /odom
        self.get_logger().info("Publishing to /odom")
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Publish to /tf
        self.get_logger().info("Publishing to /tf")
        self.tf_b = TransformBroadcaster(self)

        # Main update loop
        self.create_timer(0.05, self.loop) # 20Hz
        
    def read_encoder_delta(self, wheel: str) -> int:
        steps = self.encoder_counts[wheel]
        delta = steps - self.prev_counts[wheel]
        self.prev_counts[wheel] = steps
        return delta

    def on_encoder_counts(self, msg: Int64MultiArray):
        if len(msg.data) >= 4:
            for i, wheel in enumerate(self.robot):
                self.encoder_counts[wheel.name] = int(msg.data[i])

            if not self.prev_initialized:
                # Save count for next delta calculation
                self.prev_counts = {k: int(v) for k, v in self.encoder_counts.items()}
                self.prev_initialized = True
                self.last_t = self.get_clock().now().nanoseconds * 1e-9

    def loop(self):
        # Skip on first run
        if not self.prev_initialized:
            return
        
        # Save time for dt calculation
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_t
        if dt <= 0.0:
            return
        self.last_t = now

        speeds = {}
        for wheel in self.robot:
            delta_steps = self.read_encoder_delta(wheel.name) * (1 if wheel.forward else -1)
            revs = delta_steps / self.robot.pulses_per_rev
            speeds[wheel.name] = revs * pi * self.robot.wheel_diameter

        # Calculate velocities
        vel_x = ((speeds["FL"] + speeds["FR"] + speeds["RL"] + speeds["RR"]) / 4.0) / dt
        vel_y = -((-speeds["FL"] + speeds["FR"] + speeds["RL"] - speeds["RR"]) / 4.0) / dt
        vel_yaw = (-speeds["FL"] + speeds["FR"] - speeds["RL"] + speeds["RR"]) / (4.0 * (self.robot.robot_length + self.robot.robot_width)) / dt
        
        # Scale velocities to match real conditions
        vel_x *= 1.0
        vel_y *= 1.0
        vel_yaw *= 2.0

        # Calculate new position
        self.x += vel_x * cos(self.yaw) * dt - vel_y * sin(self.yaw) * dt
        self.y += vel_x * sin(self.yaw) * dt + vel_y * cos(self.yaw) * dt
        self.yaw += vel_yaw * dt

        # Normalize yaw to [-pi, pi]
        while self.yaw > pi:
            self.yaw -= 2 * pi
        while self.yaw < -pi:
            self.yaw += 2 * pi

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
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

        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_b.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()


