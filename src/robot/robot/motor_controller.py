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

        # Get robot details
        self.robot = Robot()

        # Motor controls
        self.motors = {}
        self.encoders = {}

        # Pids
        self.pids = {}
        self.setpoints = {"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0}

        # Pulse tracking
        self.prev_pulses = {"front_left": 0, "front_right": 0, "rear_left": 0, "rear_right": 0}

        # Map motors to controllers and set PID
        for wheel in self.robot:
            # Create controllers
            motor = Motor(forward=wheel.in1, backward=wheel.in2, pwm=False)
            ena = PWMOutputDevice(wheel.pwm, frequency=1000, initial_value=0)
            enc = RotaryEncoder(a=wheel.encoder_a, b=wheel.encoder_b, max_steps=0)

            # Save to map
            self.motors[wheel.name] = {"motor": motor, "ena": ena}
            self.encoders[wheel.name] = enc

            # Create PID
            pid = PID(**wheel.PID, setpoint=0)
            pid.output_limits = (0, 1)

            # Save to map
            self.pids[wheel.name] = pid

        self.get_logger().info('Creating subscriber on /wheel_setpoints topic')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'wheel_setpoints', self.setpoints_callback, 10
        )

        self.get_logger().info('Creating publisher on /wheel_speeds topic')
        self.wheel_state_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)

        self.get_logger().info('Creating timer for control_loop')
        self.last_time = time()
        self.create_timer(0.02, self.control_loop)

    def setpoints_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.setpoints["front_left"], self.setpoints["front_right"], self.setpoints["rear_left"], self.setpoints[
                "rear_right"] = msg.data[:4]

    def control_loop(self) -> None:
        """
        Calculates the velocity for each wheel and publishes to /wheel_speeds for odometry
        """
        # Save time for delta time calculations
        now = time()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now

        # Calculate velocity for each motor
        speeds = []
        for wheel in self.robot:
            # Get pulses
            enc = self.encoders[wheel.name]
            pulses = (-1 if wheel.forward else 1) * (enc.steps - self.prev_pulses[wheel.name])

            # Save pulses for next run
            self.prev_pulses[wheel.name] = enc.steps

            # Calculate velocity from pulses
            revs = pulses / (self.robot.pulses_per_rev * self.robot.gear_ratio)
            distance = revs * 2 * pi * (self.robot.wheel_diameter / 2)
            speed = distance / 1000 / dt

            # Update setpoint for the wheel PID
            self.pids[wheel.name].setpoint = abs(self.setpoints[wheel.name])

            # Get PID value and controllers for wheel
            duty = self.pids[wheel.name](abs(speed))
            motor = self.motors[wheel.name]["motor"]
            ena = self.motors[wheel.name]["ena"]

            # Update wheel movement based on PID
            if self.setpoints[wheel.name] >= 0.1:
                ena.value = duty
                motor.forward()
            elif self.setpoints[wheel.name] <= -0.1:
                ena.value = duty
                motor.backward()
            else:
                ena.value = 0
                motor.stop()

            # Save speed to publish
            speeds.append(speed)

        # Publish 4 wheel speed for odometry
        msg = Float32MultiArray()
        msg.data = speeds
        self.wheel_state_pub.publish(msg)

    def stop_all(self) -> None:
        """
        Stops all motors
        """
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
