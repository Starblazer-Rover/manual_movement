import rclpy
from rclpy.node import Node
import serial
import math

from std_msgs.msg import Float64MultiArray

class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, '/movement/Encoder', 1)
        timer_period = 1/30

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('opening serial')
        self.ser = serial.Serial(port='/dev/ttyACM2', baudrate=115200, timeout=1)
        self.get_logger().info('serial opened')

        self.RADIUS = 4 #inches

        self.CIRCUMFERENCE = 2 * self.RADIUS * math.pi

        self.offset = []

        data = self.parse()

        for i in range(len(data)):
            self.offset.append(float(data[i]))

    def parse(self):
        data = self.ser.read_all().decode()

        data = data.split('\r\n')

        try:
            assert len(data) > 1
        except AssertionError:
            return []

        data = data[-2]

        data = data.split(',')

        for i in range(len(data)):
            data[i] = float(data[i])

            data[i] /= 48

        if len(self.offset) < 1:
            for i in range(len(data)):
                self.offset.append(float(data[i]))

        return data

    def timer_callback(self):
        msg = Float64MultiArray()

        data = self.parse()

        try:
            assert len(data) > 1
        except AssertionError:
            return

        for i in range(len(data)):
            data[i] -= self.offset[i]

        left_distance = ((data[0] + data[1] + data[2]) / 3) * self.CIRCUMFERENCE
        right_distance = ((data[3] + data[4] + data[5]) / 3) * self.CIRCUMFERENCE

        left_distance = float(format(left_distance, ".6f"))
        right_distance = float(format(right_distance, ".6f"))

        msg.data = [left_distance, right_distance]
        
        self.publisher.publish(msg)
        
        print(f'Left: {left_distance} meters, Right: {right_distance} meters')

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