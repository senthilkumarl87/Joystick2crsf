#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <vector>
#include <map>
#include <iomanip>

struct js_event {
    uint32_t time;     /* event timestamp in milliseconds */
    int16_t value;     /* value */
    uint8_t type;      /* event type */
    uint8_t number;    /* axis/button number */
};

volatile bool running = true;

void signal_handler(int signal) {
    running = false;
}

class JoystickReader {
private:
    int fd;
    
    // Drone control values
    int roll, pitch, yaw, throttle;
    std::vector<bool> buttons;
    std::vector<int> axes;
    std::vector<std::pair<int, int>> hats;
    
    // Previous values for change detection
    int last_roll, last_pitch, last_yaw, last_throttle;
    std::vector<bool> last_buttons;
    
public:
    JoystickReader(const std::string& device) {
        fd = open(device.c_str(), O_RDONLY);
        if (fd == -1) {
            throw std::runtime_error("Could not open joystick device");
        }
        
        // Initialize values
        roll = pitch = yaw = throttle = 0;
        buttons.resize(16, false);  // Support up to 16 buttons
        axes.resize(8, 0);          // Support up to 8 axes
        hats.resize(2, {0, 0});     // Support up to 2 hats (D-pads)
        
        last_roll = last_pitch = last_yaw = last_throttle = 0;
        last_buttons.resize(16, false);
    }
    
    ~JoystickReader() {
        if (fd != -1) {
            close(fd);
        }
    }
    
    int normalize_axis(int16_t value) {
        return value / 327.67;  // Convert to -100 to 100
    }
    
    void read_events() {
        struct js_event event;
        ssize_t bytes;
        
        while ((bytes = read(fd, &event, sizeof(event))) > 0) {
            if (bytes == sizeof(event)) {
                process_event(event);
            }
        }
    }
    
    void process_event(const js_event& event) {
        switch (event.type) {
            case 0x01:  // Button event
                if (event.number < buttons.size()) {
                    buttons[event.number] = (event.value == 1);
                }
                break;
                
            case 0x02:  // Axis event
                if (event.number < axes.size()) {
                    axes[event.number] = normalize_axis(event.value);
                    update_controls();
                }
                break;
                
            case 0x03:  // Hat event (D-pad)
                if (event.number < hats.size()) {
                    hats[event.number] = std::make_pair(
                        (event.value & 0xFF), 
                        ((event.value >> 8) & 0xFF)
                    );
                }
                break;
        }
    }
    
    void update_controls() {
        // Common mapping - adjust based on your joystick
        // Left stick
        yaw = axes[0];          // Axis 0: Left stick X (left/right)
        throttle = -axes[1];     // Axis 1: Left stick Y (up/down) - inverted
        
        // Right stick  
        roll = axes[3];          // Axis 2: Right stick X (left/right)
        pitch = axes[4];         // Axis 3: Right stick Y (up/down)
        
        // Triggers (if available)
        // left_trigger = axes[4];   // Axis 4: Left trigger
        // right_trigger = axes[5];  // Axis 5: Right trigger
    }
    
    void print_status() {
        bool has_changes = false;
        
        // Check for axis changes
        if (roll != last_roll || pitch != last_pitch || 
            yaw != last_yaw || throttle != last_throttle) {
            has_changes = true;
            last_roll = roll;
            last_pitch = pitch;
            last_yaw = yaw;
            last_throttle = throttle;
        }
        
        // Check for button changes
        std::vector<int> pressed_buttons;
        for (size_t i = 0; i < buttons.size(); i++) {
            if (buttons[i] != last_buttons[i]) {
                has_changes = true;
                last_buttons[i] = buttons[i];
            }
            if (buttons[i]) {
                pressed_buttons.push_back(i);
            }
        }
        
        // Print if anything changed
        if (has_changes) {
            // Main controls
            std::cout << "ROLL: " << std::setw(4) << roll 
                      << " | PITCH: " << std::setw(4) << pitch
                      << " | YAW: " << std::setw(4) << yaw
                      << " | THROTTLE: " << std::setw(4) << throttle;
            
            // Buttons
            if (!pressed_buttons.empty()) {
                std::cout << " | BTNS: ";
                for (size_t i = 0; i < pressed_buttons.size(); i++) {
                    if (i > 0) std::cout << ",";
                    std::cout << pressed_buttons[i];
                }
            }
            
            // Triggers (aux channels)
            if (axes.size() > 4) {
                std::cout << " | L_TRIG: " << std::setw(4) << axes[4]
                          << " | R_TRIG: " << std::setw(4) << axes[5];
            }
            
            // Hats (D-pads)
            for (size_t i = 0; i < hats.size(); i++) {
                if (hats[i].first != 0 || hats[i].second != 0) {
                    std::cout << " | HAT" << i << ": (" << hats[i].first 
                              << "," << hats[i].second << ")";
                }
            }
            
            std::cout << std::endl;
        }
    }
    
    void run() {
        std::cout << "Complete Joystick Reader - Press Ctrl+C to exit" << std::endl;
        std::cout << "Shows: ROLL | PITCH | YAW | THROTTLE | BUTTONS | TRIGGERS | HATS" << std::endl;
        std::cout << "----------------------------------------------------------------" << std::endl;
        
        // Set non-blocking mode
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        
        while (running) {
            read_events();
            print_status();
            usleep(10000);  // 10ms delay
        }
    }
};

int main() {
    signal(SIGINT, signal_handler);
    
    try {
        JoystickReader reader("/dev/input/js0");
        reader.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Make sure joystick is connected and try with sudo" << std::endl;
        return 1;
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}