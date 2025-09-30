import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from .dataclasses.robot import Robot
from std_msgs.msg import Float32MultiArray


class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')

        self.robot = Robot()

        self.get_logger().info("Creating subscription on /cmd_vel topic")
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info("Creating publisher on /wheel_setpoints topic")
        self.setpoints_pub = self.create_publisher(Float32MultiArray, 'wheel_setpoints', 10)

    def cmd_vel_callback(self, msg: Twist):
        vel_x, vel_y, vel_yaw = msg.linear.x, msg.linear.y, msg.angular.z
        vel_fl = vel_x - vel_y - (self.robot.robot_length + self.robot.robot_width) * vel_yaw
        vel_fr = vel_x + vel_y + (self.robot.robot_length + self.robot.robot_width) * vel_yaw
        vel_rl = vel_x + vel_y - (self.robot.robot_length + self.robot.robot_width) * vel_yaw
        vel_rr = vel_x - vel_y + (self.robot.robot_length + self.robot.robot_width) * vel_yaw
        max_vel = max(abs(vel_fl), abs(vel_fr), abs(vel_rl), abs(vel_rr), 1.0)
        vel_fl /= max_vel
        vel_fr /= max_vel
        vel_rl /= max_vel
        vel_rr /= max_vel
        msg_out = Float32MultiArray()
        msg_out.data = [vel_fl, vel_fr, vel_rl, vel_rr]
        self.setpoints_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
