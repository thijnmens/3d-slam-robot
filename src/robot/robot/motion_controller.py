import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from .dataclasses.robot import Robot
from messages.msg import Encoder  # noqa: F401
from gpiozero import Motor, PWMOutputDevice, RotaryEncoder

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.get_logger().info("Initializing Motion Controller")

        self.get_logger().info("Creating subscription on /cmd_vel topic")
        self.cmd_vel_pub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10) # run motors based off of cmd_vel and publish encoder information 

        self.get_logger().info("Creating publisher on /encoder topic") #add launch file 
        self.encoder_pub = self.create_publisher(Encoder, '/encoder', 10) 

        self.get_logger().info("Getting robot details")
        self.robot = Robot()

        self.create_timer(0.05, self.publish_encoders)  # 20 Hz

        self.motors = {
            "FL": Motor(forward=self.robot.front_left.in1,
                        backward=self.robot.front_left.in2, pwm=False),
            "FR": Motor(forward=self.robot.front_right.in1,
                        backward=self.robot.front_right.in2, pwm=False),
            "RL": Motor(forward=self.robot.rear_left.in1,
                        backward=self.robot.rear_left.in2, pwm=False),
            "RR": Motor(forward=self.robot.rear_right.in1,
                        backward=self.robot.rear_right.in2, pwm=False),
        }
        self.enas = {
            "FL": PWMOutputDevice(self.robot.front_left.pwm, frequency=1000, initial_value=0),
            "FR": PWMOutputDevice(self.robot.front_right.pwm, frequency=1000, initial_value=0),
            "RL": PWMOutputDevice(self.robot.rear_left.pwm, frequency=1000, initial_value=0),
            "RR": PWMOutputDevice(self.robot.rear_right.pwm, frequency=1000, initial_value=0),
        }
        self.encoders = {
            "FL": RotaryEncoder(a=self.robot.front_left.encoder_a, b=self.robot.front_left.encoder_b),
            "FR": RotaryEncoder(a=self.robot.front_right.encoder_a, b=self.robot.front_right.encoder_b),
            "RL": RotaryEncoder(a=self.robot.rear_left.encoder_a, b=self.robot.rear_left.encoder_b),
            "RR": RotaryEncoder(a=self.robot.rear_right.encoder_a, b=self.robot.rear_right.encoder_b),
        }

        self.wheel_sign = {"FL": 1, "FR": -1, "RL": 1, "RR": -1}

    def on_cmd(self, twist: Twist):
        # Get pin and motor details from self.robot
        # Move the robot based on /cmd_vel
        # Return encoder details to /encoder (Check message definition
        speed = twist.linear.x
        turn = twist.angular.z

        #velocity - angular / velocity + angular 
        left_val = max(min(speed - turn, 1.0), -1.0)
        right_val = max(min(speed + turn, 1.0), -1.0)

        self._drive("FL", left_val)
        self._drive("RL", left_val)
        self._drive("FR", right_val)
        self._drive("RR", right_val)

    def publish_encoders(self):
        enc_msg = Encoder()
        enc_msg.front_left = self.wheel_sign["FL"] * int(self.encoders["FL"].steps)
        enc_msg.front_right = self.wheel_sign["FR"] * int(self.encoders["FR"].steps)
        enc_msg.rear_left = self.wheel_sign["RL"] * int(self.encoders["RL"].steps)
        enc_msg.rear_right = self.wheel_sign["RR"] * int(self.encoders["RR"].steps)

        self.encoder_pub.publish(enc_msg)

    def _drive(self, wheel: str, value: float):
        ena = self.enas[wheel]
        motor = self.motors[wheel]
        duty = abs(value)

        if value >= 0.1:
            ena.value = duty
            motor.forward()
        elif value <= -0.1:
            ena.value = duty
            motor.backward()
        else:
            ena.value = 0
            motor.stop()

def main():
    rclpy.init()
    node = MotionController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()