import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from simple_pid import PID
from time import time
from math import pi


class MotorController(Node):
	def __init__(self, pulses_per_rev=396, R=0.03):
		super().__init__('motor_controller')
		self.pulses_per_rev = pulses_per_rev
		self.R = R
		self.wheel_sign = {"FL": 1, "FR": -1, "RL": 1, "RR": -1}
		self.motors = {}
		self.encoders = {}
		self.pids = {}
		self.setpoints = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
		config = {
			"FL": {"in1": 24 , "in2": 23,  "ena": 18, "enc_a": 25,  "enc_b": 27},
			"FR": {"in1": 1, "in2": 22, "ena": 12, "enc_a": 16, "enc_b": 20},
			"RL": {"in1": 21, "in2": 26, "ena": 19, "enc_a": 0,  "enc_b": 11},
			"RR": {"in1": 6,  "in2": 5, "ena": 13, "enc_a": 9, "enc_b": 10},
		}
		self.prev_pulses = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
		for wheel, pins in config.items():
			motor = Motor(forward=pins["in1"], backward=pins["in2"], pwm=False)
			ena = PWMOutputDevice(pins["ena"], frequency=1000, initial_value=0)
			enc = RotaryEncoder(a=pins["enc_a"], b=pins["enc_b"], max_steps=0)
			self.motors[wheel] = {"motor": motor, "ena": ena}
			self.encoders[wheel] = enc
			pid = PID(1.0, 0.1, 0.05, setpoint=0)
			pid.output_limits = (0, 1)
			self.pids[wheel] = pid
		self.subscription = self.create_subscription(
			Float32MultiArray, 'wheel_setpoints', self.setpoints_callback, 10
		)
		self.wheel_state_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
		self.last_time = time()
		self.create_timer(0.02, self.control_loop)

	def setpoints_callback(self, msg: Float32MultiArray):
		if len(msg.data) >= 4:
			self.setpoints["FL"], self.setpoints["FR"], self.setpoints["RL"], self.setpoints["RR"] = msg.data[:4]

	def control_loop(self):
		now = time()
		dt = max(now - self.last_time, 1e-3)
		self.last_time = now
		speeds = []
		for wheel in ["FL", "FR", "RL", "RR"]:
			enc = self.encoders[wheel]
			pulses = self.wheel_sign[wheel] * (enc.steps - self.prev_pulses[wheel])
			self.prev_pulses[wheel] = enc.steps
			revs = pulses / self.pulses_per_rev
			distance = revs * 2 * pi * self.R
			speed = distance / dt
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
			speeds.append(speed)
		msg = Float32MultiArray()
		msg.data = speeds
		self.wheel_state_pub.publish(msg)

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
