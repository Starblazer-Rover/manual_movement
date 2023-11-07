import rclpy
import inputs
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ControllerPublisher(Node):
    def __init__(self):
        def controller_setup(self):
            self.AXES = {
                'ABS_X': 'Left Stick X',
                'ABS_Y': 'Left Stick Y',
                'ABS_RX': 'Right Stick X',
                'ABS_RY': 'Right Stick Y',
                'ABS_Z': 'LT',
                'ABS_RZ': 'RT',
                'ABS_HAT0X': 'Arrow X',
                'ABS_HAT0Y': 'Arrow Y'
            }
            self.BUTTONS = {
                'BTN_SOUTH': 'A',
                'BTN_EAST': 'B',
                'BTN_WEST': 'Y',
                'BTN_NORTH': 'X',
                'BTN_TL': 'LB',
                'BTN_TR': 'RB',
                'BTN_SELECT': 'Back',
                'BTN_START': 'Start',
                'BTN_THUMBL': 'LS',
                'BTN_THUMBR': 'RS',
            }

            self.left_stick_axis_x, self.left_stick_axis_y = 0, 0
            self.right_stick_axis_x, self.right_stick_axis_y = 0, 0
            self.left_trigger_axis, self.right_trigger_axis = 0, 0
            self.arrow_axis_x, self.arrow_axis_y = 0, 0
            self.button_axis_x, self.button_axis_y = 0, 0

            self.counter = 0

        super().__init__('controller_publisher')

        self.publisher = self.create_publisher(Int32MultiArray, 'topic', 1)
        timer_period = 0.0001
        controller_setup(self)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = [self.left_stick_axis_y, self.right_stick_axis_y, 
                    self.left_stick_axis_x, self.right_stick_axis_x,
                    self.left_trigger_axis, self.right_trigger_axis,
                    self.arrow_axis_x, self.arrow_axis_y,
                    self.button_axis_x, self.button_axis_y]

        events = inputs.get_gamepad()

        def switch_axis(name, value):
            # joysticks (up / down are inverse by default)
            if name == "Left Stick Y":
                self.left_stick_axis_y = value
            elif name == "Right Stick Y":
                self.right_stick_axis_y = value
            elif name == "Left Stick X":
                self.left_stick_axis_x = value
            elif name == "Right Stick X":
                self.right_stick_axis_x = value

            # triggers
            elif name == "LT":
                self.left_trigger_axis = value
            elif name == "RT":
                self.right_trigger_axis += value

            # arrow pad (up / down are inverse by default)
            elif name == "Arrow X":
                self.arrow_axis_x = value
            elif name == "Arrow Y":
                self.arrow_axis_y = value

        def switch_button(name, state):
            # button pad
            if name == "B" and state == 1: # ->
                self.button_axis_x = 1
            elif name == "X" and state == 1: # <-
                self.button_axis_x = 1
            elif name == "Y" and state == 1: # ^
                self.button_axis_y = 1
            elif name == "A" and state == 1: # ⌄
                self.button_axis_y = 1

            # bumper
            elif name == "LB" and state == 1:
                print(f"{name} pressed")
            elif name == "RB" and state == 1:
                print(f"{name} pressed")
            
            elif name == "LS" and state == 1:
                print(f"{name} pressed")
            elif name == "RS" and state == 1:
                print(f"{name} pressed")

            elif name == "Back" and state == 1:
                print(f"{name} pressed")
            elif name == "Start" and state == 1:
                print(f"{name} pressed")

        # check for controller events 
        for event in events:
            # axis control
            if event.code in self.AXES:
                axis_name = self.AXES[event.code]
                axis_value = int(event.state)
                switch_axis(axis_name, axis_value)
            # button control
            elif event.code in self.BUTTONS:
                button_name = self.BUTTONS[event.code]
                button_state = event.state
                switch_button(button_name, button_state)
            print(msg.data)
            
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


