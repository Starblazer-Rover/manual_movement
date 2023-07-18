import inputs

def controller_setup():
            global BUTTONS, AXES
            BUTTONS = {
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

            AXES = {
                'ABS_X': 'Left Stick X',
                'ABS_Y': 'Left Stick Y',
                'ABS_RX': 'Right Stick X',
                'ABS_RY': 'Right Stick Y',
                'ABS_Z': 'LT',
                'ABS_RZ': 'RT',
            }

def main():
    global BUTTONS, AXES

    controller_setup()
    left_axis = 0
    right_axis = 0
    while True:
           events = inputs.get_gamepad()

           for event in events:
                if event.code in AXES:
                    axis_name = AXES[event.code]
                    axis_value = int(event.state)
                    if axis_name == "Left Stick Y": 
                        left_axis = axis_value
                    elif axis_name == "Right Stick Y":
                        right_axis = axis_value
                print(f"Left Axis: {left_axis} Right Axis: {right_axis}")

if __name__ == '__main__':
     main()