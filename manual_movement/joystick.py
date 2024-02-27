import time
import paho.mqtt.client as mqtt
import pygame
import json

# Initialize Pygame and the Joystick module
pygame.init()
pygame.joystick.init()

# Check if there are any joysticks connected
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joysticks detected")
    pygame.quit()
    exit()
else:
    print(f"Detected {joystick_count} joystick(s)")

# Initialize the first connected joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Optional: Print information about joystick axes, buttons, and hats
num_axes = joystick.get_numaxes()
print(f"Number of axes: {num_axes}")

num_buttons = joystick.get_numbuttons()
print(f"Number of buttons: {num_buttons}")

num_hats = joystick.get_numhats()
print(f"Number of hats: {num_hats}")

client = mqtt.Client("rpi_joystick_client")
client.connect('192.168.68.79', 1883)
client.loop_start()

try:
    while True:
        pygame.event.pump()  # Process the event queue

        # Loop through each axis and publish its value
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            axis_topic = f"rpi/broadcast/axis{i}"
            value_str = str(axis_value)
            print(f"Publishing to {axis_topic}: {value_str}")
            client.publish(axis_topic, value_str)

        # Loop through each button and publish its state
        for i in range(joystick.get_numbuttons()):
            button_state = joystick.get_button(i)
            button_topic = f"rpi/broadcast/button{i}"
            state_str = str(button_state)
            # print(f"Publishing to {button_topic}: {state_str}")
            client.publish(button_topic, state_str)

        # Loop through each hat and publish its state
        for i in range(joystick.get_numhats()):
            hat_state = joystick.get_hat(i)
            hat_topic = f"rpi/broadcast/hat{i}"
            state_str = str(hat_state)  # hat_state is a tuple (x, y), so we convert it to string
            # print(f"Publishing to {hat_topic}: {state_str}")
            client.publish(hat_topic, state_str)

        time.sleep(0.1)  # Delay to limit the message rate

except KeyboardInterrupt:
    print("Exiting...")
    client.loop_stop()
    pygame.joystick.quit()
    pygame.quit()