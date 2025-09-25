import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from simple_pid import PID
from time import time
from math import sin, cos, pi

class MotorDriver(Node):
    def __init__(self, config, L=0.20, W=0.20, R=0.03, pulses_per_rev=396):
        super().__init__('motor_driver')

        # ROS Subscription and Publisher
        self.get_logger().info('Initializing motor driver')

        self.get_logger().info('Creating subscription on /cmd_vel topic')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Creating publisher on /odom topic')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Geometry of the Battlebot (cool guy with glasses emoji)
        self.L = L  # wheelbase
        self.W = W  # trackwidth
        self.R = R  # wheel radiusi
        self.pulses_per_rev = pulses_per_rev

        # Pose state (location of robot in a 2D coordinate system)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Timestamp since the last odometetry update (position according to velocity)
        self.last_odom_time = time()

        # Dictionary for the motor setup, the encoder and PID setup
        self.motors = {}
        self.encoders = {}
        self.pids = {}
        self.setpoints = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        self.prev_pulses = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}

        for wheel, pins in config.items():
            motor = Motor(forward=pins["in1"], backward=pins["in2"], pwm=False)
            ena   = PWMOutputDevice(pins["ena"], frequency=1000, initial_value=0)
            enc   = RotaryEncoder(a=pins["enc_a"], b=pins["enc_b"], max_steps=0)
            self.motors[wheel] = {"motor": motor, "ena": ena}
            self.encoders[wheel] = enc
            self.pids[wheel] = PID(1.0, 0.1, 0.05, setpoint=0)  # tune later
            self.pids[wheel].output_limits = (0, 1)

        # Control time - computes wheel speed with PID and Odometry time - speed of updates
        self.last_control_time = time()
        self.create_timer(0.02, self.control_loop)   # 50 Hz
        self.create_timer(0.05, self.update_odometry) # 20 Hz

    # Use cmd_vel subscription to set velocity
    def cmd_vel_callback(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        # Set the wheel linear velocity (m/s)
        v_fl = vx - vy - (self.L + self.W) * wz
        v_fr = vx + vy + (self.L + self.W) * wz
        v_rl = vx + vy - (self.L + self.W) * wz
        v_rr = vx - vy + (self.L + self.W) * wz

        # Normalize so |v| <= 1 (so we do not overspin the motors)
        max_v = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        v_fl /= max_v
        v_fr /= max_v
        v_rl /= max_v
        v_rr /= max_v

        self.setpoints.update({"FL": v_fl, "FR": v_fr, "RL": v_rl, "RR": v_rr})

    # Use encoders, so that we convert the pulses per revolution to m/s and update PID
    def control_loop(self):
        now = time()
        dt = now - self.last_control_time
        self.last_control_time = now

        for wheel in ["FL", "FR", "RL", "RR"]:
            enc = self.encoders[wheel]

            # Amount of pulses since last update
            pulses = enc.steps - self.prev_pulses[wheel]
            self.prev_pulses[wheel] = enc.steps

            # Convert pulses to m/s
            revs = pulses / self.pulses_per_rev
            distance = revs * 2 * pi * self.R
            speed = distance / dt  # m/s

            # Update the PID
            self.pids[wheel].setpoint = abs(self.setpoints[wheel])
            duty = self.pids[wheel](abs(speed))

            motor = self.motors[wheel]["motor"]
            ena   = self.motors[wheel]["ena"]

            # Apply PWM with direction
            if self.setpoints[wheel] >= 0:
                ena.value = duty
                motor.forward()
            else:
                ena.value = duty
                motor.backward()

                # 🔹 Print values for tuning
            error = self.pids[wheel].setpoint - speed
            self.get_logger().debug(
                f"[{wheel}] set={self.pids[wheel].setpoint:.2f} m/s, "
                f"measured={speed:.2f} m/s, duty={duty:.2f}, error={error:.2f}"
            )

    # -Update the odometry with the encoders
    def update_odometry(self):
        now = time()
        dt = now - self.last_odom_time
        self.last_odom_time = now

        # Movement of wheels, since last update
        d = {}
        for wheel in ["FL", "FR", "RL", "RR"]:
            pulses = self.encoders[wheel].steps - self.prev_pulses[wheel]
            revs = pulses / self.pulses_per_rev
            d[wheel] = revs * 2 * pi * self.R

        # Mecanum inverse kinematics
        vx = (d["FL"] + d["FR"] + d["RL"] + d["RR"]) / 4.0 / dt
        vy = (-d["FL"] + d["FR"] + d["RL"] - d["RR"]) / 4.0 / dt
        wz = (-d["FL"] + d["FR"] - d["RL"] + d["RR"]) / (4.0 * (self.L + self.W)) / dt

        # Calcute position of robot using the encoder information for velocity
        self.x += vx * cos(self.theta) * dt - vy * sin(self.theta) * dt
        self.y += vx * sin(self.theta) * dt + vy * cos(self.theta) * dt
        self.theta += wz * dt

        # Publish to /odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def stop_all(self):
        for wheel in self.motors:
            self.motors[wheel]["motor"].stop()
            self.motors[wheel]["ena"].value = 0.0

def main(args=None):
    rclpy.init(args=args)
    #enc-a = yellow
    config = {
        "FL": {"in1": 24 , "in2": 23,  "ena": 18, "enc_a": 25,  "enc_b": 27},
        "FR": {"in1": 1, "in2": 22, "ena": 12, "enc_a": 16, "enc_b": 20},
        "RL": {"in1": 21, "in2": 26, "ena": 19, "enc_a": 0,  "enc_b": 11},
        "RR": {"in1": 6,  "in2": 5, "ena": 13, "enc_a": 9, "enc_b": 10},
    }

    node = MotorDriver(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()


if __name__ == "__main__":
    main()

