import rclpy
import inputs
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ControllerPublisher(Node):
    def __init__(self):
        def controller_setup(self):
            self.BUTTONS = {
                'BTN_SOUTH': 'A',
                'BTN_EAST': 'B',
                'BTN_WEST': 'X',
                'BTN_NORTH': 'Y',
                'BTN_TL': 'LB',
                'BTN_TR': 'RB',
                'BTN_SELECT': 'Back',
                'BTN_START': 'Start',
                'BTN_THUMBL': 'LS',
                'BTN_THUMBR': 'RS',
            }

            self.AXES = {
                'ABS_X': 'Left Stick X',
                'ABS_Y': 'Left Stick Y',
                'ABS_RX': 'Right Stick X',
                'ABS_RY': 'Right Stick Y',
                'ABS_Z': 'LT',
                'ABS_RZ': 'RT',
            }

            self.left_axis = 0
            self.right_axis = 0
            self.counter = 0

        super().__init__('controller_publisher')

        self.publisher = self.create_publisher(Int32MultiArray, 'topic', 1)
        timer_period = 0.0001
        controller_setup(self)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = [self.left_axis, self.right_axis]


        events = inputs.get_gamepad()

        """
        for event in events:
            if event.code in self.BUTTONS:
                button_name = self.BUTTONS[event.code]
                if event.state == 1:
                    print(f"{button_name} pressed")
                else:
                    print(f"{button_name} released")
            elif event.code in self.AXES:
                axis_name = self.AXES[event.code]
                print(f"{axis_name}: {event.state}")
        """

        for event in events:
            if event.code in self.AXES:
                axis_name = self.AXES[event.code]
                axis_value = int(event.state)
                if axis_name == "Left Stick Y":
                    self.left_axis = axis_value
                elif axis_name == "Right Stick Y":
                    self.right_axis = axis_value
            
        

        if self.counter == 15:
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            self.counter = 0
        else:
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller_publisher = ControllerPublisher()
    rclpy.spin(controller_publisher)
    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


