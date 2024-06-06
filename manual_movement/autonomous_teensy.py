import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import serial


class AutonomousTeensy(Node):
    
    def __init__(self):
        super().__init__('autonomous_teensy')

        self.vector_subscriber = self.create_subscription(Float32MultiArray, '/movement/vector', self.listener_callback, 10)

        self.ser = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=1)

    def listener_callback(self, msg):
        left = msg.data[0]
        right = msg.data[1]

        string = f'{-left},{-left},{-left},{right},{right},{right}\n'
        print(string)

        self.ser.write(string.encode())


def main(args=None):
    rclpy.init(args=args)

    vector_subscriber = AutonomousTeensy()

    try:
        rclpy.spin(vector_subscriber)
    except KeyboardInterrupt:
        vector_subscriber.ser.write('0,0,0,0,0,0\n'.encode())
        vector_subscriber.ser.close()
        pass
    finally:
        vector_subscriber.destroy_node()


if __name__ == '__main__':
    main()