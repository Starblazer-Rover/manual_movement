import serial
import sys
import time

def main():

    try:
        roll_angle = sys.argv[1]
        pitch_angle = sys.argv[2]

        print(roll_angle)
        print(pitch_angle)
    except IndexError:
        print('python3 arm_autolevel.py <roll_angle> <pitch angle>')

    ser = serial.Serial(port='/dev/ttyACM3', baudrate=115200, timeout=1)

    print('serial opened')

    while True:

        ser.write('Axis 0: 0.0\n'.encode())
        ser.write('Axis 1: 0.0\n'.encode())
        ser.write('Axis 2: 0.0\n'.encode())
        ser.write('Axis 3: 0.0\n'.encode())

        ser.write('Button 0: 0.0\n'.encode())
        ser.write('Button 1: 0.0\n'.encode())
        
        ser.write('Hat 0: (0, 0)\n'.encode())

        ser.write('AutoLevelingStatus: 1\n'.encode())
        ser.write(f'RollAngle: {roll_angle}\n'.encode())
        ser.write(f'PitchAngle: {pitch_angle}\n'.encode())
        ser.write(f'ControlMode: 0\n'.encode())

        print('sent')

        time.sleep(0.05)


if __name__ == '__main__':
    main()
    