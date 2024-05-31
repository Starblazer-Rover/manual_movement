import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import pygame


class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')

        pygame.init()
        pygame.joystick.init()

        self.joystick = pygame.joystick.Joystick(1)
        self.joystick.init()

        self.publisher = self.create_publisher(Float32MultiArray, '/movement/joystick', 10)
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if pygame.joystick.get_count() <= 0:
            print('No joysticks detected')
            data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher.publish(Float32MultiArray(data=data))

            return

        pygame.event.pump()

        msg = Float32MultiArray()

        #Axis 0, Axis 1, Axis 2, Axis 3, Button 0, Button 1, Hat 1
        data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            axis_value = self.joystick.get_axis(i)
            data[i] = axis_value

        for i in range(2):
            button_state = self.joystick.get_button(i)
            data[i+4] = float(button_state)

        hat_state = self.joystick.get_hat(0)
        data[6] = float(hat_state[0])
        data[7] = float(hat_state[1])

        print(data)

        print('-----------------')

        msg.data = data

        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    joystick_publisher = JoystickPublisher()

    try:
        rclpy.spin(joystick_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.joystick.quit()
        pygame.quit()

        joystick_publisher.destroy_node()


if __name__ == '__main__':
    main()