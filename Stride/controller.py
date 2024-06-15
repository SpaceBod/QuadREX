# controller.py
import pygame
import sys

def init_joystick():
    # Initialize Pygame
    pygame.init()

    # Initialize the joysticks
    pygame.joystick.init()

    # Check if a joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick connected")
        sys.exit()

    # Get the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Joystick name: {}".format(joystick.get_name()))
    return joystick

def print_axes(joystick):
    axes_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

    left_stick = [axes_values[0], axes_values[1]]  # Left stick: x, y
    right_stick = [axes_values[2], axes_values[3]]  # Right stick: x, y
    triggers = [axes_values[4], axes_values[5]]  # Right and left trigger

    if abs(left_stick[0]) > 0.2 or abs(left_stick[1]) > 0.2:
        print(f"Left Stick: {left_stick}")
    if abs(right_stick[0]) > 0.2 or abs(right_stick[1]) > 0.2:
        print(f"Right Stick: {right_stick}")
    print(f"Triggers: {triggers}")

    return left_stick, right_stick, triggers