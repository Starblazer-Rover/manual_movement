import rclpy
import inputs
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ControllerPublisher(Node):
    def __init__(self):
        def controller_setup(self):
            self.KEYS = {
                'ABS_X': 'Left Stick X',
                'ABS_Y': 'Left Stick Y',
                'ABS_RX': 'Right Stick X',
                'ABS_RY': 'Right Stick Y',
                'ABS_Z': 'LT',
                'ABS_RZ': 'RT',
                'ABS_HAT0X': 'Arrow X',
                'ABS_HAT0Y': 'Arrow Y',
                'BTN_SOUTH': 'A',
                'BTN_EAST': 'B',
                'BTN_NORTH': 'X',
                'BTN_WEST': 'Y',
                'BTN_TL': 'LB',
                'BTN_TR': 'RB',
                'BTN_SELECT': 'Back',
                'BTN_START': 'Start',
                'BTN_THUMBL': 'LS',
                'BTN_THUMBR': 'RS',
            }

            self.left_stick_y, self.right_stick_y = 0, 0 
            self.left_stick_x, self.right_stick_x = 0, 0
            self.left_trigger, self.right_trigger = 0, 0
            self.left_bumper, self.right_bumper = 0, 0
            self.button_y, self.button_a = 0, 0
            self.button_x, self.button_b = 0, 0
            self.arrow_y_up, self.arrow_y_down = 0, 0
            self.arrow_x_left, self.arrow_x_right = 0, 0

            self.counter = 0

        super().__init__('controller_publisher')

        self.publisher = self.create_publisher(Int32MultiArray, '/movement/Controller', 1)
        timer_period = 0.0001
        controller_setup(self)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # input controls from game controller 
        events = inputs.get_gamepad()
        '''
        Data Documentation:
            msg.data[0] = left_stick_y: left joystick, up/down 
            msg.data[1] = right_stick_y: right joystick, up/down 
            msg.data[2] = left_stick_x: left joystick, left/right 
            msg.data[3] = right_stick_x: right joystick, left/right 
            msg.data[4] = left_trigger: left trigger 
            msg.data[5] = right_trigger: right trigger
            msg.data[6] = left_bumper: left bumper 
            msg.data[7] = right_bumper: right bumper 
            msg.data[8] = button_y: button y on pad (up) 
            msg.data[9] = button_a: button a on pad (down) 
            msg.data[10] = button_x: button x on pad (left)
            msg.data[11] = button_b: button b on pad (right) 
            msg.data[12] = arrow_y_up:
            msg.data[13] = arrow_y_down: 
            msg.data[14] = arrow_x_left: 
            msg.data[15] = arrow_x_right: 

            General scheme: up, down, left, right
            Value ranges:
                joystick range = [?, ?]
                trigger range = [?, ?]
                buttons = 1
        '''
        # message array 
        msg = Int32MultiArray()
        msg.data = [-self.left_stick_y, -self.right_stick_y, 
                    self.left_stick_x, self.right_stick_x,
                    self.left_trigger, self.right_trigger,
                    self.left_bumper, self.right_bumper,
                    self.button_y, self.button_a,
                    self.button_x, self.button_b,
                    self.arrow_y_up, self.arrow_y_down,
                    self.arrow_x_left, self.arrow_x_right]

        # switch statement to update message array
        def switch(name, value):
            # joysticks (up / down are inverse by default)
            self.left_stick_y = value if name == 'Left Stick Y' else self.left_stick_y
            self.right_stick_y = value if name == 'Right Stick Y' else self.right_stick_y
            self.left_stick_x = value if name == 'Left Stick X' else self.left_stick_x
            self.right_stick_x = value if name == 'Right Stick X' else self.left_stick_y
            # triggers
            self.left_trigger = value if name == 'LT' else self.left_trigger
            self.right_trigger = value if name == 'RT' else self.right_trigger
            # bumper buttons
            self.left_bumper = 1 if name == 'LB' and value == 1 else 0
            self.right_bumper = 1 if name == 'RB' and value == 1 else 0
            # button pad
            self.button_y = 1 if name == 'Y' and value == 1 else 0
            self.button_a = 1 if name == 'A' and value == 1 else 0
            self.button_x = 1 if name == 'X' and value == 1 else 0
            self.button_b = 1 if name == 'B' and value == 1 else 0
            # arrow pad (up / down are inverse by default)
            self.arrow_y_up = 1 if name == 'Arrow Y' and value > 0 else 0
            self.arrow_y_down = 1 if name == 'Arrow Y' and value < 0 else 0
            self.arrow_x_left = 1 if name == 'Arrow X' and value < 0 else 0
            self.arrow_x_right = 1 if name == 'Arrow X' and value > 0 else 0

        # Check for controller events | unused buttons: 'Back', 'start', 'LS, 'RS'
        for event in events:
            if event.code in self.KEYS:
                axis_name = self.KEYS[event.code]
                axis_value = int(event.state)
                switch(axis_name, axis_value)
            print(msg.data)
        
        # publish message
        if self.counter == 15:
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            self.counter = 0
        else:
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller_publisher = ControllerPublisher()

    try:
        rclpy.spin(controller_publisher)
    except KeyboardInterrupt:
        msg = Int32MultiArray()

        msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        controller_publisher.publisher.publish(msg)

    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


