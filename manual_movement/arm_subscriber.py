import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float32MultiArray

class ArmSubscriber(Node):

    def __init__(self):
        super().__init__('arm_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, '/movement/arm', self.listener_callback, 1)
        self.subscription

        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

    def listener_callback(self, msg):
        
        print(msg.data)
        for i in range(len(msg.data)):
            print(i)
            self.ser.write(f'Axis {i}: {msg.data[i]}\n'.encode())
def main(args=None):
    rclpy.init(args=args)

    arm_subscriber = ArmSubscriber()

    try:
        rclpy.spin(arm_subscriber)
    except KeyboardInterrupt:
        arm_subscriber.ser.close()
        arm_subscriber.destroy_node()

if __name__ == '__main__':
    main()