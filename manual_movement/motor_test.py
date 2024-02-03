from roboclaw_3 import Roboclaw
import time

def main():
    try:
        roboclaw_1 = Roboclaw("/dev/ttyACM0", 38400)
        print(roboclaw_1.Open())

        roboclaw_2 = Roboclaw("/dev/ttyACM1", 38400)
        print(roboclaw_2.Open())

        roboclaw_1.ForwardM1(129, 50)
        roboclaw_2.ForwardM2(128, 50)

        time.sleep(2)

        roboclaw_1.ForwardM1(129, 0)
        roboclaw_2.ForwardM2(128, 0)

        del roboclaw_1
        del roboclaw_2

    except KeyboardInterrupt:
        roboclaw_1.ForwardM1(129, 0)
        roboclaw_2.ForwardM2(128, 0)

if __name__ == '__main__':
    main()
