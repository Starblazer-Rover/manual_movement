import rclpy
from roboclaw_3 import Roboclaw
from manual_movement.motor_control import MotorControl
from manual_movement.manual_movement.encoders import Encoder
from rclpy.executors import MultiThreadedExecutor


def __initialize_motors():
    roboclaw_1 = Roboclaw("/dev/ttyACM0", 38400)
    roboclaw_2 = Roboclaw("/dev/ttyACM1", 38400)

    roboclaw_1.Open()
    roboclaw_2.Open()

    return roboclaw_1, roboclaw_2


def main(args=None):
    rclpy.init(args=args)

    roboclaw_1, roboclaw_2 = __initialize_motors()

    motor_control = MotorControl(roboclaw_1, roboclaw_2)
    encoder = Encoder(roboclaw_1, roboclaw_2)

    executor = MultiThreadedExecutor()

    executor.add_node(motor_control)
    #executor.add_node(encoder)

    try:
        executor.spin()

    except KeyboardInterrupt:
        motor_control.roboclaw_1.ForwardM1(0x81, 0)
        motor_control.roboclaw_2.ForwardM2(0x80, 0)

        motor_control.destroy_node()
        encoder.destroy_node()
        

if __name__ == '__main__':
    main()
