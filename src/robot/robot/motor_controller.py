from types import SimpleNamespace
from typing import Dict, List, Set

from lark.utils import Enumerator

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from messages.msg import Encoder  # noqa: F401

from .dataclasses.robot import Robot
from gpiozero import Motor, PWMOutputDevice, RotaryEncoder

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.get_logger().info('Initializing motor controller')

        self.get_logger().info("Creating subscription on /cmd_vel topic")
        self.cmd_vel_pub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel,
                                                    10)  # run motors based off of cmd_vel and publish encoder information

        self.get_logger().info("Creating publisher on /encoder topic")  # add launch file
        self.encoder_pub = self.create_publisher(Encoder, '/encoder', 10)

        self.get_logger().info("Getting robot details")
        self.robot = Robot()

        self.create_timer(0.05, self.publish_encoder) # 20 Hz

        # Create motors
        self.motors = {}
        for wheel in self.robot:
            self.motors[wheel.name] = Motor(forward=wheel.in1, backward=wheel.in2, enable=wheel.pwm, pwm=True)

        # Create encoders
        self.encoders = {}
        for wheel in self.robot:
            self.encoders[wheel.name] = RotaryEncoder(a=wheel.encoder_a, b=wheel.encoder_b, wrap=True)

    def on_cmd_vel(self, twist: Twist):
        vel_x = twist.linear.x
        vel_y = twist.linear.y
        vel_yaw = twist.angular.z

        # Calculate velocities for each motor
        wheel_radius = self.robot.wheel_diameter / 2
        half_length = self.robot.robot_length / 2
        half_width = self.robot.robot_width / 2

        wheel_speeds = [(vel_x - vel_y - (half_length + half_width) * vel_yaw) / wheel_radius,
        (vel_x + vel_y + (half_length + half_width) * vel_yaw) / wheel_radius,
        (vel_x + vel_y - (half_length + half_width) * vel_yaw) / wheel_radius,
        (vel_x - vel_y + (half_length + half_width) * vel_yaw) / wheel_radius ]

        # Map angular speed linearly to PWM [-1, 1]
        # Assume self.robot.max_wheel_speed is the max angular speed (rad/s)
        max_w = max(self.robot.max_wheel_speed, 1e-6)
        pwms = [max(min(w / max_w, 1.0), -1.0) for w in wheel_speeds]

        for i, wheel in enumerate(self.robot):
            self._drive(wheel.name, pwms[i])

    def _drive(self, wheel_name: str, value: float):

        motor: Motor = self.motors[wheel_name]
        # Apply per-wheel forward inversion if needed
        signed = value if getattr(self.robot, wheel_name).forward else -value
        # Clamp to [-1, 1]
        signed = max(min(signed, 1.0), -1.0)

        if signed >= 0.1:
            motor.forward(signed)
        elif signed <= -0.1:
            motor.backward(abs(signed))
        else:
            motor.stop()

    def publish_encoder(self):
        encoder = Encoder()
        for wheel in self.robot:
            setattr(encoder, wheel.name, self.encoders[wheel.name].steps)

        self.encoder_pub.publish(encoder)

def main():
    rclpy.init()
    node = MotorController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()