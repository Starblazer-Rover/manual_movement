import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float32MultiArray


class ArmSubscriber(Node):

    def __init__(self):
        super().__init__('arm_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, '/movement/joystick', self.listener_callback, 1)

        self.ser = serial.Serial(port='/dev/ttyACM2', baudrate=115200, timeout=1)

    def listener_callback(self, msg):
        
        for i in range(4):
            string = f'Axis {i}: {msg.data[i]}\n'
            print(string)
            self.ser.write(string.encode())
            
        for i in range(2):
            string = f'Button {i}: {msg.data[i+4]}\n'
            self.ser.write(string.encode())

        string = f'Hat 0: ({int(msg.data[6])}, {int(msg.data[7])})\n'
        
        self.ser.write(string.encode())


        self.ser.write(f'AutoLevelingStatus: 0\n'.encode())
        self.ser.write(f'RollAngle: 0.0\n'.encode())
        self.ser.write(f'PitchAngle: 0.0\n'.encode())
        self.ser.write(f'ControlMode: 0\n'.encode())


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