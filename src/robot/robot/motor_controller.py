from math import pi
from time import time

from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from simple_pid import PID

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .dataclasses.robot import Robot


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Get robot data
        self.robot = Robot()

        # Motor classes
        self.motors = {}
        self.encoders = {}
        self.pids = {}

        # PID requirements
        self.setpoints = {"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0}
        self.prev_pulses = {"front_left": 0, "front_right": 0, "rear_left": 0, "rear_right": 0}

        # Configure motor driving classes
        for wheel in self.robot:
            motor = Motor(forward=wheel.in1, backward=wheel.in2, pwm=False)
            ena = PWMOutputDevice(wheel.pwm, frequency=1000, initial_value=0)
            enc = RotaryEncoder(a=wheel.encoder_a, b=wheel.encoder_b, max_steps=0)

            self.motors[wheel.name] = {"motor": motor, "ena": ena}
            self.encoders[wheel.name] = enc

            pid = PID(1.0, 0.1, 0.05, setpoint=0)
            pid.output_limits = (0, 1)

            self.pids[wheel.name] = pid

        self.get_logger().info("Creating subscription on /wheel_setpoints topic")
        self.wheel_setpoints_sub = self.create_subscription(
            Float32MultiArray, '/wheel_setpoints', self.setpoints_callback, 10
        )

        self.get_logger().info("Creating publisher on /wheel_speeds topic")
        self.wheel_speeds_pub = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)

        self.last_time = time()

        self.create_timer(0.02, self.control_loop)

    def setpoints_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.setpoints["front_left"], self.setpoints["front_right"], self.setpoints["right_left"], self.setpoints["rear_left"] = msg.data[:4]

    def control_loop(self):
        now = time()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        speeds = []
        for wheel in self.robot:

            enc = self.encoders[wheel.name]
            pulses = wheel.forward * (enc.steps - self.prev_pulses[wheel.name])
            self.prev_pulses[wheel.name] = enc.steps

            revs = pulses / self.robot.pulses_per_rev
            distance = revs * 2 * pi * (self.robot.wheel_diameter / 2)
            speed = distance / dt

            self.pids[wheel.name].setpoint = abs(self.setpoints[wheel.name])

            duty = self.pids[wheel.name](abs(speed))
            motor = self.motors[wheel.name]["motor"]
            ena = self.motors[wheel.name]["ena"]

            if self.setpoints[wheel.name] >= 0.1:
                ena.value = duty
                motor.forward()
            elif self.setpoints[wheel.name] <= -0.1:
                ena.value = duty
                motor.backward()
            else:
                ena.value = 0
                motor.stop()
            speeds.append(speed)

        msg = Float32MultiArray()
        msg.data = speeds
        self.wheel_speeds_pub.publish(msg)

    def stop_all(self):
        for wheel in self.motors:
            self.motors[wheel]["motor"].stop()
            self.motors[wheel]["ena"].value = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()
