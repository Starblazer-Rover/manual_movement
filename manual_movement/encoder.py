import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class Encoder(Node):

    def __init__(self, roboclaw_1, roboclaw_2):
        super().__init__('encoder_publisher')

        self.__publisher = self.create_publisher(Int32MultiArray, '/movement/Encoder', 10)

        timer_period = 1/200

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.roboclaw_1 = roboclaw_1
        self.roboclaw_2 = roboclaw_2

        self.roboclaw_1.SetEncM1(0x81, 0)
        self.roboclaw_2.SetEncM2(0x80, 0)

    def timer_callback(self):
        # Number of pulses per revolution
        PPR = 256

        # 8 in. Diameter Wheels
        CIRCUMFERENCE = 8 * np.pi

        left_encoder = self.roboclaw_1.ReadEncM1(0x81)
        right_encoder = self.roboclaw_2.ReadEncM2(0x80)

        left_distance = (left_encoder / PPR) * CIRCUMFERENCE
        right_distance = (right_encoder / PPR) * CIRCUMFERENCE

        msg = Int32MultiArray()

        msg.data[0] = left_distance
        msg.data[1] = right_distance

        self.__publisher.publish(msg)

        self.roboclaw_1.SetEncM1(0x81, 0)
        self.roboclaw_2.SetEncM2(0x80, 0)

