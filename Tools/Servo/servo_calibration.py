from adafruit_servokit import ServoKit
import time
import signal
import json
import os

# Create the servo kit instance
kit = ServoKit(channels=16)

# Load angles from JSON if available
def load_offsets_from_json():
    if os.path.exists('servo_offsets.json'):
        with open('servo_offsets.json', 'r') as json_file:
            return json.load(json_file)
    else:
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

angles = load_offsets_from_json()

# Setting the pulse width range for servos 0 to 11
for i in range(12):
    kit.servo[i].set_pulse_width_range(500, 2500)

def signal_handler(sig, frame):
    # Function to handle the SIGINT (Ctrl+C)
    print("Ctrl+C pressed, disconnecting servos...")
    for i in range(12):
        kit.servo[i].angle = None  # This command will deactivate the servo
    print("Servos disconnected. Exiting...")
    exit(0)  # Exit the program

# Register the SIGINT handler
signal.signal(signal.SIGINT, signal_handler)

def set_all_servos_to_90():
    # Move all servos (0 to 11) to 90 degrees
    for i in range(12):
        kit.servo[i].angle = 90 + angles[i]
        time.sleep(0.1)  # small delay to allow the servo to move

def calibrate_servo(servo_number):
    while True:
        try:
            offset = input(f"Enter the angle offset for servo {servo_number} (or 'y' to finish): ")
            if offset.lower() == 'y':
                break
            offset = int(offset)
            angles[servo_number] = offset
            kit.servo[servo_number].angle = 90 + angles[servo_number]
        except ValueError:
            print("Please enter a valid integer offset or 'y' to finish.")

def save_offsets_to_json():
    with open('servo_offsets.json', 'w') as json_file:
        json.dump(angles, json_file)
    print("Servo offsets saved to 'servo_offsets.json'.")

def main():
    while True:
        try:
            servo_number = int(input("Enter the servo number (0-11) to calibrate (or -1 to exit): "))
            if servo_number == -1:
                break
            if 0 <= servo_number < 12:
                calibrate_servo(servo_number)
            else:
                print("Please enter a valid servo number between 0 and 11.")
        except ValueError:
            print("Please enter a valid integer.")

    save_offsets_to_json()

# Continuously set all servos to 90 until interrupted
try:
    while True:
        set_all_servos_to_90()
        main()
        time.sleep(0.01)  # wait for a second before setting them again
except Exception as e:
    print(f"An error occurred: {e}")
