import rclpy
from rclpy.node import Node

import can
import struct
import time

from std_msgs.msg import Int32MultiArray


class NeoSubscriber(Node):

    """
    0x02: 041
    0x03: 061
    0x04: 081
    0x05: 0a1
    0x06: 0c1
    0x07: 0e1

    Order:
    0x04
    0x02
    0x06
    0x05
    0x03
    """
    def __init__(self):
        self.MAX_SPEED = 24
        self.MAX_VALUE = 32768
        self.ID_ORDER = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        self.LEFT_ID_ORDER = [0x07, 0x02, 0x03]
        self.RIGHT_ID_ORDER = [0x04, 0x06, 0x05]

        self.bus = can.interface.Bus("can0", bustype="socketcan")

        self.close_loop()

        super().__init__('neo_subscriber')
        self.subscription = self.create_subscription(Int32MultiArray, '/movement/Controller', self.listener_callback, 1)
        self.subscription

    def close_loop(self):
        for id in self.ID_ORDER:
            self.bus.send(can.Message(
                arbitration_id=(id << 5 | 0x07), # 0x07: Set_Axis_State
                data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                is_extended_id=False
            ))

        print("Initializing Motors")

    def move_left(self, velocity):
        counter = 0
        for msg in self.bus:
            if msg.arbitration_id == (self.LEFT_ID_ORDER[counter] << 5 | 0x01): # 0x01: Heartbeat
                _, state, __, ___ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state != 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                    continue
            else:
                continue

            self.bus.send(can.Message(
                arbitration_id=(self.LEFT_ID_ORDER[counter] << 5 | 0x0d), # 0x0d: Set_Input_Vel
                data=struct.pack('<ff', float(velocity), 0.0), # 1.0: velocity, 0.0: torque feedforward
                is_extended_id=False
            ))

            counter += 1

            if counter >= len(self.LEFT_ID_ORDER):
                break

    def move_right(self, velocity):
        counter = 0
        for msg in self.bus:
            if msg.arbitration_id == (self.RIGHT_ID_ORDER[counter] << 5 | 0x01): # 0x01: Heartbeat
                _, state, __, ___ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state != 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                    continue
            else:
                continue

            self.bus.send(can.Message(
                arbitration_id=(self.RIGHT_ID_ORDER[counter] << 5 | 0x0d), # 0x0d: Set_Input_Vel
                data=struct.pack('<ff', float(velocity), 0.0), # 1.0: velocity, 0.0: torque feedforward
                is_extended_id=False
            ))

            counter += 1

            if counter >= len(self.RIGHT_ID_ORDER):
                break

    def listener_callback(self, msg):
        left_axis = msg[0]
        right_axis = msg[1]

        left_ratio = left_axis / self.MAX_VALUE
        right_ratio = right_axis / self.MAX_VALUE

        left_velocity = left_ratio * self.MAX_SPEED
        right_velocity = right_ratio * self.MAX_SPEED

        self.move_left(left_velocity)
        self.move_right(right_velocity)


def main(args=None):
    rclpy.init(args=args)

    neo_subscriber = NeoSubscriber()

    try:
        rclpy.spin(neo_subscriber)

    except KeyboardInterrupt:
        neo_subscriber.move_left(0)
        neo_subscriber.move_right(0)

    neo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




