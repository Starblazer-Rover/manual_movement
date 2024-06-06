import rclpy
from rclpy.node import Node
import serial
import sys

from std_msgs.msg import Float32MultiArray


class ArmSubscriber(Node):

    def __init__(self, mode, status, roll, pitch):
        super().__init__('arm_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, '/movement/joystick', self.listener_callback, 1)

        self.mode = mode
        self.status = status
        self.roll = roll
        self.pitch = pitch

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


        self.ser.write(f'AutoLevelingStatus: {self.status}\n'.encode())
        self.ser.write(f'RollAngle: {self.roll}\n'.encode())
        self.ser.write(f'PitchAngle: {self.pitch}\n'.encode())
        self.ser.write(f'ControlMode: {self.mode}\n'.encode())


def main(args=None):
    rclpy.init(args=args)

    try:
        mode = sys.argv[1]
        status = sys.argv[2]
        roll = sys.argv[3]
        pitch = sys.argv[4]

        print(f'Autoleveling - Roll: {roll}, Pitch: {pitch}')
    except IndexError:
        print('Manual Movement')

        mode = 0
        status = 0
        roll = 0.0
        pitch = 0.0

    arm_subscriber = ArmSubscriber(mode, status, roll, pitch)

    try:
        rclpy.spin(arm_subscriber)
    except KeyboardInterrupt:
        arm_subscriber.ser.close()
        arm_subscriber.destroy_node()

if __name__ == '__main__':
    main()