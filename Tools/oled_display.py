#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Displays a static 64x64 square, representing an eye, on two SH1106 OLED displays.
"""

import os
from PIL import Image, ImageDraw
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.sprite_system import framerate_regulator

# Setup for dual SH1106 displays
def get_devices(addresses=[0x3C, 0x3D]):
    serials = [i2c(port=1, address=address) for address in addresses]
    return [sh1106(serial, rotate=0) for serial in serials]

def main():
    devices = get_devices()
    regulator = framerate_regulator(fps=10)
    width, height = 128, 64  # Display dimensions

    # Parameters for the eye
    eye_size = 64
    pos = (width - eye_size) // 2  # Center the eye horizontally

    for device in devices:
        with regulator:
            frame = Image.new("RGB", device.size, "white")
            draw = ImageDraw.Draw(frame)
            top_left = (pos, (height - eye_size) // 2)  # Center the eye vertically as well
            draw.rectangle([top_left, (top_left[0] + eye_size, top_left[1] + eye_size)], fill="black")
            device.display(frame.convert(device.mode))
    while True:
    	continue
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated.")
