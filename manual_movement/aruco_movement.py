import rclpy
from rclpy.node import Node

import can
import struct
import time

from std_msgs.msg import Int64


class ArucoSubscriber(Node):

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
    0x07
    0x03
    """
    def __init__(self):
        self.SPEED_RATIO = 25
        self.CENTER = 370

        self.ID_ORDER = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        self.LEFT_ID_ORDER = [0x07, 0x02, 0x03]
        self.RIGHT_ID_ORDER = [0x04, 0x06, 0x05]

        self.counter = 0

        self.bus = can.interface.Bus("can0", bustype="socketcan")

        self.close_loop()

        super().__init__('aruco_subscriber')
        self.subscription = self.create_subscription(Int64, '/movement/aruco', self.listener_callback, 10)
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
                print("got")
                if state != 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                    continue
            else:
                continue
            self.bus.send(can.Message(
                arbitration_id=(self.LEFT_ID_ORDER[counter] << 5 | 0x0d), # 0x0d: Set_Input_Vel
                data=struct.pack('<ff', -float(velocity), 0.0), # 1.0: velocity, 0.0: torque feedforward
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
        x_coord = msg.data

        if self.counter == 60 and x_coord == 1000:
            self.move_left(5)
            self.move_right(-5)

            print("Spinning")

        elif x_coord == 1000:
            self.counter += 1
        
        else:
            self.counter = 0

            difference = self.CENTER - x_coord

            if abs(difference) <= 50:
                self.move_left(5)
                self.move_right(5)
                print("Moving Forward")
            else:
                self.move_left(-difference / self.SPEED_RATIO)
                self.move_right(difference / self.SPEED_RATIO)

                print("Moving Left/Right")




def main(args=None):
    rclpy.init(args=args)

    aruco_subscriber = ArucoSubscriber()

    try:
        rclpy.spin(aruco_subscriber)

    except KeyboardInterrupt:
        aruco_subscriber.move_left(0)
        aruco_subscriber.move_right(0)

    aruco_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

