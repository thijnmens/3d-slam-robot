import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice
from .dataclasses.robot import Robot
from std_msgs.msg import Int64MultiArray
from simple_pid import PID as SimplePID
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info("Initializing motor_controller")

        # Get robot information
        self.robot = Robot()

        # Create motor objects
        self.motors = {}
        for wheel in self.robot:
            motor = Motor(forward=wheel.in1, backward=wheel.in2, pwm=False)
            ena = PWMOutputDevice(wheel.pwm, frequency=1000, initial_value=0.0)
            self.motors[wheel.name] = {"motor": motor, "ena": ena}

        # Subscribe to cmd_vel
        self.get_logger().info("Subscribing to /cmd_vel")
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # Subscribe to encoder counts
        self.get_logger().info("Subscribing to /encoder_counts")
        self.create_subscription(Int64MultiArray, 'encoder_counts', self.on_encoder_counts, 10)

        # Current setpoints per wheel in [-1, 1]
        self.setpoints = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}

        # Keep track of encoder counts
        self.prev_counts = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self.encoder_counts = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self.prev_initialized = False
        self.last_t = time.time()

        # Create PID controllers
        self.pids = {}
        for wheel in self.robot:
            kp, ki, kd = wheel.pid
            pid = SimplePID(kp, ki, kd, setpoint=0)
            pid.output_limits = (-1.0, 1.0)
            # set integrator windup guard similar to previous i_clamp
            pid.integrator_limit = 0.3
            self.pids[wheel.name] = pid

        # Control timer (20ms)
        self.create_timer(0.02, self.control_loop)

    def on_cmd_vel(self, msg: Twist):
        factor = (self.robot.robot_length + self.robot.robot_width) * msg.angular.z

        vel_fl = msg.linear.x - msg.linear.y - factor
        vel_fr = msg.linear.x + msg.linear.y + factor
        vel_rl = msg.linear.x + msg.linear.y - factor
        vel_rr = msg.linear.x - msg.linear.y + factor

        # Get max abs velocity
        max_vel = max(abs(vel_fl), abs(vel_fr), abs(vel_rl), abs(vel_rr), 0.7)

        # Normalize and clamp between 0.3 and 1
        def clamp_and_scale(v):
            norm = v / max_vel
            sign = 1 if norm >= 0 else -1
            norm_abs = abs(norm)
            norm_abs = max(0.4, min(norm_abs, 0.7))
            return sign * norm_abs

        self.setpoints.update({
            "FL": clamp_and_scale(vel_fl),
            "FR": clamp_and_scale(vel_fr),
            "RL": clamp_and_scale(vel_rl),
            "RR": clamp_and_scale(vel_rr),
        })

    def on_encoder_counts(self, msg: Int64MultiArray):
        # Check if full message sent
        if len(msg.data) >= 4:

            # Update encoder counts
            for i, wheel in enumerate(self.robot):
                self.encoder_counts[wheel.name] = int(msg.data[i])

            # initialize prev_counts on first received message to avoid large deltas
            if not self.prev_initialized:
                self.prev_counts = {k: int(v) for k, v in self.encoder_counts.items()}
                self.prev_initialized = True

    def drive(self, wheel: str, value: float):
        motor = self.motors[wheel]["motor"]
        ena = self.motors[wheel]["ena"]

        # Don't drive if speed is too low
        speed = abs(value)
        if speed <= 0.05:
            ena.value = 0.0
            motor.stop()
            return
        
        # Apply inversion to direction
        duty = self.robot.duty_min + self.robot.duty_gain * speed
        
        # Clamp duty to [0.0, 1.0]
        duty = max(0.0, min(duty, 1.0))

        # Apply duty to motors
        ena.value = duty
        if value >= 0.0:
            motor.forward()
        else:
            motor.backward()

    def control_loop(self):
        now = time.time()
        dt = now - self.last_t if self.last_t is not None else 0.02
        if dt <= 0.0:
            return
        self.last_t = now

        # Calculate speed per wheel
        measured_norm = {}
        for wheel in self.robot:
            steps = int(self.encoder_counts.get(wheel.name, 0))

            delta = steps - int(self.prev_counts.get(wheel.name, 0))
            delta *= 1 if wheel.forward else -1

            self.prev_counts[wheel.name] = steps

            revolutions = float(delta) / float(self.robot.pulses_per_rev)
            distance = revolutions * math.pi * self.robot.wheel_diameter
            speed = distance / dt

            measured_norm[wheel.name] = speed / self.robot.max_wheel_speed

        # run PID per-wheel to get normalized command and drive
        for wheel in self.robot:
            setpoint = self.setpoints[wheel.name]

            # Reset PID and motor if setpoint close to 0
            if abs(setpoint) < 0.02:
                self.pids[wheel.name].reset()
                self.drive(wheel.name, 0.0)
                continue
            
            # Set speed to 0 if close to 0
            speed = measured_norm[wheel.name]
            if abs(speed) < 0.02:
                speed = 0.0
            
            pid = self.pids[wheel.name]
            pid.setpoint = setpoint

            vel = pid(speed)

            # Ignore if output is close to 0
            if abs(vel) < 0.02:
                vel = 0.0
            
            self.drive(wheel.name, vel)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure motors off
        for wheel in node.robot:
            node.motors[wheel.name]["ena"].value = 0.0
            node.motors[wheel.name]["motor"].stop()
        node.destroy_node()

if __name__ == '__main__':
    main()


