import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial

class SerialConnector(Node):
    def __init__(self):
        super(SerialConnector, self).__init__("serial_connector")

        self.get_logger().info("Starting serial connection with arduino")

        self.get_logger().info("creating reader for /cmd_vel")
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info("Connecting to serial port")
        self.arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)

        self.get_logger().info("Creating timer")
        self.create_timer(0.01, self.read_serial)


    def on_cmd_vel(self, msg: Twist):
        # Check if serial port is opened, if not, open it
        if not self.arduino.is_open:
            self.arduino.open()

        self.get_logger().info('sending cmd_vel')

        self.arduino.write(bytearray([
            1, 1, 1, 1, # Motor directions
            1, 1, 1, 1 # Motor speed
        ]))

    def read_serial(self):
        # Check if serial port is opened, if not, open it
        if not self.arduino.is_open:
            self.arduino.open()

        if self.arduino.in_waiting >= 5:
            self.get_logger().info(self.arduino.read(5).hex())

def main(args=None):
    rclpy.init(args=args)
    node = SerialConnector()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
