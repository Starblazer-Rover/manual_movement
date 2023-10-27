from roboclaw_3 import Roboclaw
import time

def main():
    try:
        roboclaw_1 = Roboclaw("/dev/ttyACM0", 38400)
        roboclaw_1.Open()

        roboclaw_2 = Roboclaw("/dev/ttyACM1", 38400)
        roboclaw_2.Open()

        roboclaw_1.ForwardM1(0x80, 25)
        print("1")
        roboclaw_1.ForwardM2(0x80, 25)
        print("2")
        roboclaw_2.ForwardM1(0x81, 25)
        print("3")
        roboclaw_2.ForwardM2(0x81, 25)
        print("4")

        time.sleep(2)

        roboclaw_1.ForwardM1(0x80, 0)
        roboclaw_1.ForwardM2(0x80, 0)
        roboclaw_2.ForwardM1(0x81, 0)
        roboclaw_2.ForwardM2(0x81, 0)

        del roboclaw_1
        del roboclaw_2

    except KeyboardInterrupt:
        roboclaw_1.ForwardM1(0x80, 0)
        roboclaw_1.ForwardM2(0x80, 0)
        roboclaw_2.ForwardM1(0x81, 0)
        roboclaw_2.ForwardM2(0x81, 0)

if __name__ == '__main__':
    main()
