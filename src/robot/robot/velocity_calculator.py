import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class VelocityCalculator(Node):
    def __init__(self, L=0.20, W=0.21):
        super().__init__('velocity_calculator')
        self.L = L
        self.W = W
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.setpoints_pub = self.create_publisher(Float32MultiArray, 'wheel_setpoints', 10)

    def cmd_vel_callback(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        v_fl = vx - vy - (self.L + self.W) * wz
        v_fr = vx + vy + (self.L + self.W) * wz
        v_rl = vx + vy - (self.L + self.W) * wz
        v_rr = vx - vy + (self.L + self.W) * wz
        max_v = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        v_fl /= max_v
        v_fr /= max_v
        v_rl /= max_v
        v_rr /= max_v
        msg_out = Float32MultiArray()
        msg_out.data = [v_fl, v_fr, v_rl, v_rr]
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
