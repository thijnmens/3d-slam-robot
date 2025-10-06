import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from simple_pid import PID
from time import time
from math import pi


class MotorController(Node):
    def __init__(self, config, L=0.20, W=0.21, R=0.03, pulses_per_rev=396):
        super().__init__('motor_controller')

        # Geometry
        self.L = L
        self.W = W
        self.R = R
        self.pulses_per_rev = pulses_per_rev

        # Sign convention (right motors inverted in hardware)
        self.wheel_sign = {"FL": 1, "FR": -1, "RL": 1, "RR": -1}

        # Hardware and control
        self.motors = {}
        self.encoders = {}
        self.pids = {}
        self.setpoints = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        self.prev_pulses_control = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}

        for wheel, pins in config.items():
            motor = Motor(forward=pins["in1"], backward=pins["in2"], pwm=False)
            ena = PWMOutputDevice(pins["ena"], frequency=1000, initial_value=0)
            enc = RotaryEncoder(a=pins["enc_a"], b=pins["enc_b"], max_steps=0)
            self.motors[wheel] = {"motor": motor, "ena": ena}
            self.encoders[wheel] = enc
            pid = PID(1.0, 0.1, 0.05, setpoint=0)
            pid.output_limits = (0, 1)
            self.pids[wheel] = pid

        # ROS I/O
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 10)

        # Timers
        self.last_control_time = time()
        self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.create_timer(0.05, self.publish_ticks)  # 20 Hz

        self.get_logger().info('Motor controller initialized')

    def cmd_vel_callback(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        # Inverse kinematics for mecanum (normalized to <= 1.0)
        v_fl = vx - vy - (self.L + self.W) * wz
        v_fr = vx + vy + (self.L + self.W) * wz
        v_rl = vx + vy - (self.L + self.W) * wz
        v_rr = vx - vy + (self.L + self.W) * wz

        max_v = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        self.setpoints.update({
            "FL": v_fl / max_v,
            "FR": v_fr / max_v,
            "RL": v_rl / max_v,
            "RR": v_rr / max_v,
        })

    def control_loop(self):
        now = time()
        dt = now - self.last_control_time
        if dt <= 0:
            return
        self.last_control_time = now

        for wheel in ["FL", "FR", "RL", "RR"]:
            enc = self.encoders[wheel]

            # Pulses since last control update
            pulses = self.wheel_sign[wheel] * (enc.steps - self.prev_pulses_control[wheel])
            self.prev_pulses_control[wheel] = enc.steps

            # Convert pulses to linear speed (m/s)
            revs = pulses / self.pulses_per_rev
            distance = revs * 2 * pi * self.R
            speed = distance / dt

            # PID on absolute speed, sign handled by direction
            self.pids[wheel].setpoint = abs(self.setpoints[wheel])
            duty = self.pids[wheel](abs(speed))

            motor = self.motors[wheel]["motor"]
            ena = self.motors[wheel]["ena"]

            if self.setpoints[wheel] >= 0.1:
                ena.value = duty
                motor.forward()
            elif self.setpoints[wheel] <= -0.1:
                ena.value = duty
                motor.backward()
            else:
                ena.value = 0
                motor.stop()

    def publish_ticks(self):
        # Publish cumulative encoder steps (with sign convention applied)
        ticks = Int32MultiArray()
        ticks.data = [
            int(self.wheel_sign["FL"] * self.encoders["FL"].steps),
            int(self.wheel_sign["FR"] * self.encoders["FR"].steps),
            int(self.wheel_sign["RL"] * self.encoders["RL"].steps),
            int(self.wheel_sign["RR"] * self.encoders["RR"].steps),
        ]
        self.ticks_pub.publish(ticks)

    def stop_all(self):
        for wheel in self.motors:
            self.motors[wheel]["motor"].stop()
            self.motors[wheel]["ena"].value = 0.0


def main(args=None):
    rclpy.init(args=args)

    # Pin configuration stays here with the hardware node
    config = {
        "FL": {"in1": 24, "in2": 23, "ena": 18, "enc_a": 25, "enc_b": 27},
        "FR": {"in1": 1, "in2": 22, "ena": 12, "enc_a": 16, "enc_b": 20},
        "RL": {"in1": 21, "in2": 26, "ena": 19, "enc_a": 0, "enc_b": 11},
        "RR": {"in1": 6, "in2": 5, "ena": 13, "enc_a": 9, "enc_b": 10},
    }

    node = MotorController(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

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
            pid = PID(*wheel.PID, setpoint=0)
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
        self.create_timer(0.01, self.control_loop)

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
            distance = revs * pi * self.robot.wheel_diameter
            speed = distance / dt

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
