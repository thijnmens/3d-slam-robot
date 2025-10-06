import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .dataclasses.robot import Robot


class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')

        # Get robot details
        self.robot = Robot()

        self.get_logger().info("Creating subscription on /cmd_vel topic")
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info("Creating publisher on /wheel_setpoints topic")
        self.setpoints_pub = self.create_publisher(Float32MultiArray, 'wheel_setpoints', 10)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Calculates the velocities of the wheel to match the requested cmd_vel vector
        :param msg: cmd_vel vector
        """

        # Unpack requested vector
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        # Calculate velocity for each wheel to result in desired vector
        vel_front_left = vx - vy - (self.robot.robot_length + self.robot.robot_width) * wz
        vel_front_right = vx + vy + (self.robot.robot_length + self.robot.robot_width) * wz
        vel_rear_left = vx + vy - (self.robot.robot_length + self.robot.robot_width) * wz
        vel_rear_right = vx - vy + (self.robot.robot_length + self.robot.robot_width) * wz

        # Convert velocity to value between 0 and 1
        max_vel = max(abs(vel_front_left), abs(vel_front_right), abs(vel_rear_left), abs(vel_rear_right), 1.0)

        # Limit velocities to maximum
        vel_front_left /= max_vel
        vel_front_right /= max_vel
        vel_rear_left /= max_vel
        vel_rear_right /= max_vel

        # Publish calculated velocities
        msg_out = Float32MultiArray()
        msg_out.data = [vel_front_left, vel_front_right, vel_rear_left, vel_rear_right]
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
