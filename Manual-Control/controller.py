import pygame
import sys
import time
import numpy as np
from movement.quadruped import QuadrupedRobot

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

def scale_value(value, src_range, dst_range):
    src_min, src_max = src_range
    dst_min, dst_max = dst_range
    if value < src_min:
        value = src_min
    elif value > src_max:
        value = src_max
    return dst_min + (float(value - src_min) / float(src_max - src_min) * (dst_max - dst_min))

def calculate_angle(x_axis, y_axis, tolerance=0.1):
    if abs(x_axis) < tolerance and abs(y_axis) < tolerance:
        return 0  # Neutral position
    angle = np.degrees(np.arctan2(-y_axis, x_axis))
    return angle - 90

def calculate_leg_length(x_axis, y_axis, tolerance=0.1):
    magnitude = np.sqrt(x_axis**2 + y_axis**2)
    if magnitude < tolerance:
        return 0  # Neutral position
    return scale_value(magnitude, [0, 1], [0, 0.34])

def calculate_orientation(x_axis, limit, tolerance=0.1):
    if abs(x_axis) < tolerance:
        return 0  # Neutral position
    return scale_value(x_axis, [-1, 1], [-limit, limit])

def calculate_leg_rotation(x_axis, tolerance=0.1):
    if abs(x_axis) < tolerance:
        return 0  # Neutral position
    return scale_value(x_axis, [-1, 1], [-0.2, 0.2])

if __name__ == "__main__":
    joystick = init_joystick()
    robot = QuadrupedRobot()
    mode = "default"  # Initial mode

    # Initialize height and position variables
    height = 0
    position_x = 0
    increment = 0.002

    # Flags for button states
    lb_pressed = False
    rb_pressed = False
    dpad_up_pressed = False
    dpad_down_pressed = False
    dpad_left_pressed = False
    dpad_right_pressed = False

    # Example of setting initial parameters
    robot.set_body_position([position_x, 0, height])
    robot.set_body_orientation([0, 0, 0])
    robot.set_gait_parameters(
        leg_length=0,
        angle=0,
        leg_rotation=0,
        step_period=0.4,
        step_duration_asymmetry=0,
        offset=[0.5, 0.0, 0.0, 0.5]
    )

    step_period = 0.4  # Initialize step_period

    try:
        while True:
            pygame.event.pump()
            left_stick_x = joystick.get_axis(0)
            left_stick_y = joystick.get_axis(1)
            right_stick_x = joystick.get_axis(2)
            right_stick_y = joystick.get_axis(3)
            button_b = joystick.get_button(1)

            # Bumpers for height control
            button_lb = joystick.get_button(6)  # LB
            button_rb = joystick.get_button(7)  # RB

            # Adjust height based on bumper input
            if button_lb and not lb_pressed:
                height -= increment
                if height < -0.02:
                    height = -0.02
                print(f"Decreasing height to: {height}")
                lb_pressed = True
            elif not button_lb:
                lb_pressed = False

            if button_rb and not rb_pressed:
                height += increment
                if height > 0.02:
                    height = 0.02
                print(f"Increasing height to: {height}")
                rb_pressed = True
            elif not button_rb:
                rb_pressed = False

            # Check DPAD input for step_period and position_x adjustment
            hat = joystick.get_hat(0)
            if hat[1] == 1 and not dpad_up_pressed:  # DPAD up
                step_period += 0.05
                print("Increasing step period to:", step_period)
                dpad_up_pressed = True
            elif hat[1] != 1:
                dpad_up_pressed = False

            if hat[1] == -1 and not dpad_down_pressed:  # DPAD down
                step_period -= 0.05
                if step_period < 0.05:  # Prevent step period from becoming negative or too small
                    step_period = 0.05
                print("Decreasing step period to:", step_period)
                dpad_down_pressed = True
            elif hat[1] != -1:
                dpad_down_pressed = False

            if hat[0] == -1 and not dpad_left_pressed:  # DPAD left
                position_x -= increment
                if position_x < -0.02:
                    position_x = -0.02
                print(f"Decreasing position_x to: {position_x}")
                dpad_left_pressed = True
            elif hat[0] != -1:
                dpad_left_pressed = False

            if hat[0] == 1 and not dpad_right_pressed:  # DPAD right
                position_x += increment
                if position_x > 0.02:
                    position_x = 0.02
                print(f"Increasing position_x to: {position_x}")
                dpad_right_pressed = True
            elif hat[0] != 1:
                dpad_right_pressed = False

            # Toggle mode when 'B' button is pressed
            if button_b:
                mode = "leg_rotation" if mode == "default" else "default"
                print(f"Mode changed to: {mode}")
                time.sleep(0.3)  # Debounce delay to avoid multiple toggles on a single press

            # Apply tolerance to the joystick input to avoid jitter
            if abs(left_stick_x) < 0.1:
                left_stick_x = 0
            if abs(left_stick_y) < 0.1:
                left_stick_y = 0
            if abs(right_stick_x) < 0.1:
                right_stick_x = 0
            if abs(right_stick_y) < 0.1:
                right_stick_y = 0

            if mode == "default":
                # Calculate the angle based on the joystick position
                angle = calculate_angle(left_stick_x, left_stick_y)
                # Calculate leg length based on the magnitude of the joystick input
                leg_length = calculate_leg_length(left_stick_x, left_stick_y)
                # Calculate orientation based on the right joystick X-axis
                roll = calculate_orientation(right_stick_x, 0.25)
                pitch = calculate_orientation(right_stick_y, 0.15)
                leg_rotation = 0
            else:
                angle = 0
                leg_length = 0.04
                leg_rotation = calculate_leg_rotation(right_stick_x)
                roll = 0
                pitch = 0

            # Update robot parameters based on the calculated values
            robot.update_gait_parameters({
                'leg_length': leg_length,
                'angle': angle,
                'leg_rotation': leg_rotation,
                'step_period': step_period,
                'step_duration_asymmetry': 0,
                'offset': [0.5, 0.0, 0.0, 0.5]
            })

            robot.update_body_position([position_x, 0, height])
            robot.update_body_orientation([roll, pitch, 0])

            robot.step()
            time.sleep(0.02)  # Small delay to prevent excessive CPU usage

    except KeyboardInterrupt:
        print("Exiting...")
