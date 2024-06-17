import pygame
import sys

def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick connected")
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Joystick name: {}".format(joystick.get_name()))
    return joystick

if __name__ == "__main__":
    joystick = init_joystick()
    print("Joystick initialized successfully")
