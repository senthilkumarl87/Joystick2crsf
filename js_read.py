import struct
import os

# Open joystick device
joystick_device = '/dev/input/js0'

try:
    with open(joystick_device, 'rb') as js:
        print("Reading joystick input... Press Ctrl+C to stop")
        while True:
            event = js.read(8)
            time, value, type, number = struct.unpack('IhBB', event)
            print(f"Time: {time}, Value: {value}, Type: {type}, Number: {number}")
except FileNotFoundError:
    print(f"Joystick device {joystick_device} not found")
except KeyboardInterrupt:
    print("Exiting...")