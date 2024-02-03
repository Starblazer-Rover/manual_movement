import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from roboclaw_3 import Roboclaw


class MotorControl(Node):
    def __init__(self, roboclaw_1, roboclaw_2):
        
        self.MAX_VALUE = 32768

        super().__init__('motor_control')
        self.subscription = self.create_subscription(Int32MultiArray, '/movement/Controller', self.listener_callback, 1)

        self.roboclaw_1 = roboclaw_1

        self.roboclaw_2 = roboclaw_2

    def listener_callback(self, msg):
        
        left_axis = msg.data[0]
        right_axis = msg.data[1]
        self.get_logger().info(f"Left_Axis: {left_axis}, Right_Axis: {right_axis}")

        if left_axis < 0:
            left_axis = -left_axis
            ratio = left_axis / self.MAX_VALUE
            left_axis = int(ratio * 30)

            self.roboclaw_2.ForwardM2(0x80, left_axis)
            
        elif left_axis > 0:
            ratio = left_axis / self.MAX_VALUE
            left_axis = int(ratio * 30)

            self.roboclaw_2.BackwardM2(0x80, left_axis)
        
        if right_axis < 0:
            right_axis = -right_axis
            ratio = right_axis / self.MAX_VALUE
            right_axis = int(ratio * 30)

            self.roboclaw_1.ForwardM1(0x81, right_axis)

        elif right_axis > 0:
            ratio = right_axis / self.MAX_VALUE
            right_axis = int(ratio * 30)

            self.roboclaw_1.BackwardM1(0x81, right_axis)
