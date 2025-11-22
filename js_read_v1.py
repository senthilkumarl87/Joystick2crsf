import struct
import os

class RC_data: 
    def __init__(self):
        self.channels = [1500] * 16  # Initialize 16 channels with neutral value

def read_joystick_interpreted():
    joystick_device = '/dev/input/js0'
    
    try:
        with open(joystick_device, 'rb') as js:
            print("Reading joystick input... Press Ctrl+C to stop")
            print("Format: [TYPE] Number: Value -> Description")
            print("-" * 50)
            
            while True:
                event = js.read(8)
                time, value, type_code, number = struct.unpack('IhBB', event)
                
                # Interpret the event
                if type_code == 0x01:  # Button event
                    state = "PRESSED" if value == 1 else "RELEASED"
                    print(f"[BUTTON] Button {number}: {value} -> {state}")
                    
                elif type_code == 0x02:  # Axis event
                    # Common axis mappings
                    axis_names = {
                        0: "Left Stick X",
                        1: "Left Stick Y", 
                        2: "Right Stick X",
                        3: "Right Stick Y",
                        4: "Left Trigger",
                        5: "Right Trigger"
                    }
                    axis_name = axis_names.get(number, f"Unknown Axis {number}")
                    
                    # Normalize axis value to -1.0 to 1.0 for easier reading
                    normalized = value / 32767.0
                    print(f"[AXIS] {axis_name} ({number}): {value} -> {normalized:.3f}")
                    
                    
                    
                elif type_code == 0x03:  # Initial state
                    print(f"[INIT] Device {number}: {value}")
                    
    except FileNotFoundError:
        print(f"Joystick device {joystick_device} not found")
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    read_joystick_interpreted()