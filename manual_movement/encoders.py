import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Int32MultiArray

class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')

        self.publisher = self.create_publisher(Int32MultiArray, '/movement/Encoder', 1)
        timer_period = 1/30

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('opening serial')
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
        self.get_logger().info('serial opened')

    def timer_callback(self):
        data = self.ser.read_all().decode()

        print(data)

def main(args=None):
    rclpy.init(args=args)

    encoder_publisher = EncoderPublisher()

    try:
        rclpy.spin(encoder_publisher)
    except KeyboardInterrupt:
        encoder_publisher.ser.close()
        encoder_publisher.destroy_node()

if __name__ == '__main__':
    main()