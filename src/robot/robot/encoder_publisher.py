import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from gpiozero import RotaryEncoder


class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')

        # create encoders (hardware pins)
        self.encoders = {
            "FL": RotaryEncoder(a=25, b=27, max_steps=0),
            "FR": RotaryEncoder(a=16, b=20, max_steps=0),
            "RL": RotaryEncoder(a=0,  b=11, max_steps=0),
            "RR": RotaryEncoder(a=9,  b=10, max_steps=0),
        }

        self.pub = self.create_publisher(Int64MultiArray, 'encoder_counts', 10)
        self.create_timer(0.02, self.loop)

    def loop(self):
        # publish cumulative steps as an Int64MultiArray in order [FL, FR, RL, RR]
        msg = Int64MultiArray()
        msg.data = [
            int(self.encoders['FL'].steps),
            int(self.encoders['FR'].steps),
            int(self.encoders['RL'].steps),
            int(self.encoders['RR'].steps),
        ]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
