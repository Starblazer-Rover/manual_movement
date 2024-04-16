import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Int32MultiArray

class NeoSubscriber(Node):

    def __init__(self):
        super().__init__('arm_subscriber')
        self.subscription = self.create_subscription(Int32MultiArray, '/movement/Controller', self.listener_callback, 1)
        self.subscription

        self.MAX_VALUE = 32768

        self.get_logger().info('opening serial')
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
        self.get_logger().info('serial opened')

    def listener_callback(self, msg):
        
        
        left = (msg.data[0] / self.MAX_VALUE) * 30
        right = (msg.data[1] / self.MAX_VALUE) * 30



        self.get_logger().info(f'Left: {left}, Right: {right}')

        self.ser.write(f'{-left},{-left},{-left},{right},{right},{right}\n'.encode())

        self.get_logger().info('finished writing')

def main(args=None):
    rclpy.init(args=args)

    neo_subscriber = NeoSubscriber()

    try:
        rclpy.spin(neo_subscriber)
    except KeyboardInterrupt:
        neo_subscriber.ser.close()
        neo_subscriber.destroy_node()

if __name__ == '__main__':
    main()