import pygame
import time
import math

class JoystickToDrone:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            raise Exception("No joystick detected!")
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        print(f"Joystick: {self.joystick.get_name()}")
        print("Drone RC Controller Ready - Move joystick to see output")
        print("Channel Mapping:")
        print("Left Stick: Throttle (up/down), Yaw (left/right)")
        print("Right Stick: Roll (left/right), Pitch (forward/back)")
        print("-" * 80)
        
        # RC channel ranges (typical PWM values)
        self.MIN_PWM = 1000
        self.MID_PWM = 1500
        self.MAX_PWM = 2000
        
        # Dead zone to prevent drift
        self.DEAD_ZONE = 0.1
        
        # Store previous values to detect changes
        self.prev_commands = None
        self.prev_buttons = None
        
    def apply_dead_zone(self, value):
        """Apply dead zone to prevent small movements from affecting output"""
        if abs(value) < self.DEAD_ZONE:
            return 0.0
        return value
    
    def map_to_pwm(self, value, invert=False):
        """Map joystick value (-1 to 1) to PWM range (1000-2000)"""
        if invert:
            value = -value
            
        value = self.apply_dead_zone(value)
        
        # Map from [-1, 1] to [MIN_PWM, MAX_PWM]
        pwm = self.MID_PWM + (value * (self.MAX_PWM - self.MIN_PWM) / 2)
        return int(pwm)
    
    def has_changed(self, current_commands, current_buttons):
        """Check if joystick state has changed significantly"""
        if self.prev_commands is None or self.prev_buttons is None:
            return True
            
        # Check if any axis values changed beyond threshold
        for channel in ['roll', 'pitch', 'throttle', 'yaw']:
            if abs(current_commands[channel] - self.prev_commands[channel]) > 5:  # 5 PWM units threshold
                return True
        
        # Check if any buttons changed
        if current_buttons != self.prev_buttons:
            return True
            
        return False
    
    def get_drone_commands(self):
        """Convert joystick input to drone RC commands"""
        pygame.event.pump()
        
        # Get all axes and buttons
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        
        # Common joystick mapping for drones
        if len(axes) >= 4:
            left_stick_x = axes[0]   # Channel 4: Yaw
            left_stick_y = axes[1]   # Channel 3: Throttle (inverted)
            right_stick_x = axes[2]  # Channel 1: Roll
            right_stick_y = axes[3]  # Channel 2: Pitch
        else:
            left_stick_x, left_stick_y, right_stick_x, right_stick_y = 0, 0, 0, 0
        
        # Create RC commands dictionary
        rc_commands = {
            'roll': self.map_to_pwm(right_stick_x),      # Channel 1
            'pitch': self.map_to_pwm(right_stick_y),     # Channel 2  
            'throttle': self.map_to_pwm(left_stick_y, invert=True),  # Channel 3 (inverted)
            'yaw': self.map_to_pwm(left_stick_x),        # Channel 4
            
            # Auxiliary channels from buttons
            'aux1': self.MIN_PWM if buttons[0] else self.MAX_PWM if buttons[1] else self.MID_PWM,
            'aux2': self.MIN_PWM if buttons[2] else self.MAX_PWM if buttons[3] else self.MID_PWM,
        }
        
        return rc_commands, buttons, hats
    
    def print_commands(self, commands, buttons, hats):
        """Display the RC commands when joystick is used"""
        # Create button status string
        active_buttons = [f"Btn{i}" for i, pressed in enumerate(buttons) if pressed]
        button_str = f" | Buttons: {active_buttons}" if active_buttons else ""
        
        # Create hat status string
        active_hats = [f"Hat{i}:{hat}" for i, hat in enumerate(hats) if hat != (0, 0)]
        hat_str = f" | Hats: {active_hats}" if active_hats else ""
        
        print(f"ROLL: {commands['roll']:4d} | PITCH: {commands['pitch']:4d} | "
              f"THROTTLE: {commands['throttle']:4d} | YAW: {commands['yaw']:4d} | "
              f"AUX1: {commands['aux1']:4d} | AUX2: {commands['aux2']:4d}"
              f"{button_str}{hat_str}")
    
    def run(self):
        """Main loop to read and convert joystick input - only print when used"""
        try:
            while True:
                commands, buttons, hats = self.get_drone_commands()
                
                # Only print if something changed
                if self.has_changed(commands, buttons):
                    self.print_commands(commands, buttons, hats)
                    self.prev_commands = commands
                    self.prev_buttons = buttons.copy()
                
                time.sleep(0.05)  # 20Hz update rate
                
        except KeyboardInterrupt:
            print("\nExiting Drone Controller...")
        finally:
            pygame.quit()

# Simpler version that prints raw axis changes
class SimpleJoystickMonitor:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            raise Exception("No joystick detected!")
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        print(f"Joystick: {self.joystick.get_name()}")
        print("Simple Joystick Monitor - Only prints when joystick is used")
        print("Move sticks, press buttons, or use D-pad to see output")
        print("-" * 60)
        
        self.prev_axes = None
        self.prev_buttons = None
        self.prev_hats = None
        
    def has_changes(self, axes, buttons, hats):
        """Check if any input has changed"""
        if self.prev_axes is None:
            return True
            
        # Check axes (with dead zone)
        for i, (curr, prev) in enumerate(zip(axes, self.prev_axes)):
            if abs(curr - prev) > 0.01:  # Small threshold for axes
                return True
        
        # Check buttons
        if buttons != self.prev_buttons:
            return True
            
        # Check hats
        if hats != self.prev_hats:
            return True
            
        return False
    
    def run_simple(self):
        """Simple monitoring that only prints when joystick is used"""
        try:
            while True:
                pygame.event.pump()
                
                # Get current state
                axes = [round(self.joystick.get_axis(i), 2) for i in range(self.joystick.get_numaxes())]
                buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
                
                # Only print if something changed
                if self.has_changes(axes, buttons, hats):
                    # Build output string
                    output_parts = []
                    
                    # Add active axes
                    active_axes = [f"Axis{i}:{val}" for i, val in enumerate(axes) if abs(val) > 0.1]
                    if active_axes:
                        output_parts.append(" | ".join(active_axes))
                    
                    # Add active buttons
                    active_buttons = [f"Btn{i}" for i, pressed in enumerate(buttons) if pressed]
                    if active_buttons:
                        output_parts.append(f"Buttons: {active_buttons}")
                    
                    # Add active hats
                    active_hats = [f"Hat{i}:{hat}" for i, hat in enumerate(hats) if hat != (0, 0)]
                    if active_hats:
                        output_parts.append(f"Hats: {active_hats}")
                    
                    # Print if there's any activity
                    if output_parts:
                        print(" | ".join(output_parts))
                    
                    # Update previous state
                    self.prev_axes = axes
                    self.prev_buttons = buttons.copy()
                    self.prev_hats = hats.copy()
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nExiting monitor...")
        finally:
            pygame.quit()

if __name__ == "__main__":
    try:
        # Choose which mode you want:
        
        # Option 1: Drone RC commands (only prints when sticks move)
        print("Starting Drone RC Controller...")
        controller = JoystickToDrone()
        controller.run()
        
        # Option 2: Simple raw monitoring (only prints when joystick is used)
        # print("Starting Simple Joystick Monitor...")
        # monitor = SimpleJoystickMonitor()
        # monitor.run_simple()
        
    except Exception as e:
        print(f"Error: {e}")