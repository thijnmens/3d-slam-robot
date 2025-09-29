from geometry_msgs.msg import Twist
from rclpy import Node
from src.robot.robot.dataclasses.robot import Robot


class MotionController(Node):
    def __init__(self, config):
        super().__init__('motion_controller')

        self.get_logger().info("Initializing Motion Controller")

        self.get_logger().info("Creating subscription on /cmd_vel topic")
        self.cmd_vel_pub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.get_logger().info("Creating publisher on /encoder topic")
        self.encoder_pub = self.create_publisher(Twist, '/encoder', 10)

        self.get_logger().info("Getting robot details")
        self.robot = Robot()

    def on_cmd(self, twist: Twist):
        # Get pin and motor details from self.robot
        # Move the robot based on /cmd_vel
        # Return encoder details to /encoder (Check message definition)
        pass


