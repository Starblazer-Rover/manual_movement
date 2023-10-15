import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from roboclaw_python.roboclaw_python.roboclaw_3 import Roboclaw


class RoverMovement(Node):
    def __init__(self):
        
        self.MAX_VALUE = 32768


        super().__init__('rover_movement')
        self.subscription = self.create_subscription(Int32MultiArray, 'topic', self.listener_callback, 1)
        self.subscription

        self.roboclaw_1 = Roboclaw("/dev/ttyACM0", 38400)
        self.roboclaw_1.Open()

        self.roboclaw_2 = Roboclaw("/dev/ttyACM1", 38400)
        self.roboclaw_2.Open()

    def listener_callback(self, msg):
        left_axis = msg.data[0]
        right_axis = msg.data[1]

        if left_axis < 0:
            left_axis = -left_axis
            ratio = left_axis / self.MAX_VALUE
            left_axis = int(ratio * 30)

            self.roboclaw_2.BackwardM1(0x81, left_axis)
            self.roboclaw_1.BackwardM2(0x80, left_axis)
            
        elif left_axis > 0:
            ratio = left_axis / self.MAX_VALUE
            left_axis = int(ratio * 30)

            self.roboclaw_2.ForwardM1(0x81, left_axis)
            self.roboclaw_1.ForwardM2(0x80, left_axis)

        
        if right_axis < 0:
            right_axis = -right_axis
            ratio = right_axis / self.MAX_VALUE
            right_axis = int(ratio * 30)

            self.roboclaw_2.BackwardM2(0x81, right_axis)
            self.roboclaw_1.BackwardM1(0x80, right_axis)

        elif right_axis > 0:
            ratio = right_axis / self.MAX_VALUE
            right_axis = int(ratio * 30)

            self.roboclaw_2.ForwardM2(0x81, right_axis)
            self.roboclaw_1.ForwardM1(0x80, right_axis)

def main(args=None):
    try:
        rclpy.init(args=args)

        rover_movement = RoverMovement()

        rclpy.spin(rover_movement)

        rover_movement.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        rover_movement.roboclaw_1.ForwardM1(0x80, 0)
        rover_movement.roboclaw_1.ForwardM2(0x80, 0)
        rover_movement.roboclaw_2.ForwardM1(0x81, 0)
        rover_movement.roboclaw_2.ForwardM2(0x81, 0)

if __name__ == '__main__':
    main()
