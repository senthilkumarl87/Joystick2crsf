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
#include <string>
#include <stdexcept>

struct js_event {
    uint32_t time;
    int16_t value;
    uint8_t type;
    uint8_t number;
};

volatile bool running = true;

void signal_handler(int signal) {
    running = false;
}

class AdvancedJoystickReader {
private:
    int fd;
    
    // Main flight controls
    int roll, pitch, yaw, throttle;
    
    // AUX channels (for flight modes, etc.)
    int aux1, aux2, aux3, aux4;
    
    // Raw inputs
    std::vector<bool> buttons;
    std::vector<int> axes;
    std::vector<std::pair<int, int>> hats;
    
    // Flight mode mapping
    std::map<int, std::string> flight_modes = {
        {1000, "MANUAL"},
        {1250, "ACRO"},
        {1500, "STABILIZE"},
        {1750, "ALT_HOLD"},
        {2000, "AUTO"}
    };

public:
    AdvancedJoystickReader(const std::string& device) {
        fd = open(device.c_str(), O_RDONLY);
        if (fd == -1) {
            throw std::runtime_error("Could not open joystick device");
        }
        
        // Initialize all values
        roll = pitch = yaw = throttle = 0;
        aux1 = aux2 = aux3 = aux4 = 1500;  // Center PWM values
        
        buttons.resize(20, false);
        axes.resize(10, 0);
        hats.resize(2, {0, 0});
        
        // Set non-blocking mode
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        
        std::cout << "Joystick initialized successfully" << std::endl;
        std::cout << "Axis Mapping: " << std::endl;
        std::cout << "  Axis 0: YAW (Left stick X)" << std::endl;
        std::cout << "  Axis 1: THROTTLE (Left stick Y - inverted)" << std::endl;
        std::cout << "  Axis 3: ROLL (Right stick X)" << std::endl;
        std::cout << "  Axis 4: PITCH (Right stick Y)" << std::endl;
    }
    
    ~AdvancedJoystickReader() {
        if (fd != -1) close(fd);
    }
    
    int normalize_axis(int16_t value) {
        return value / 327.67;  // Convert to -100 to 100
    }
    
    int axis_to_pwm(int16_t value) {
        // Convert axis to standard PWM range (1000-2000)
        return 1500 + (value / 32.767);
    }
    
    void process_event(const js_event& event) {
        switch (event.type) {
            case 0x01:  // Button event
                if (event.number < buttons.size()) {
                    buttons[event.number] = (event.value == 1);
                    update_aux_channels();
                }
                break;
                
            case 0x02:  // Axis event
                if (event.number < axes.size()) {
                    axes[event.number] = event.value;
                    update_controls();
                }
                break;
                
            case 0x03:  // Hat event
                if (event.number < hats.size()) {
                    hats[event.number] = {event.value & 0xFF, (event.value >> 8) & 0xFF};
                    update_aux_channels();
                }
                break;
        }
    }
    
    void update_controls() {
        // Updated axis mapping as requested
        yaw = normalize_axis(axes[0]);          // Axis 0: Left stick X (left/right)
        throttle = normalize_axis(-axes[1]);    // Axis 1: Left stick Y (up/down) - inverted
        
        // Right stick  
        roll = normalize_axis(axes[3]);         // Axis 3: Right stick X (left/right)
        pitch = normalize_axis(axes[4]);        // Axis 4: Right stick Y (up/down)
        
        // Print debug info when axes are updated
        static int last_axes[5] = {0};
        for (int i : {0, 1, 3, 4}) {
            if (axes[i] != last_axes[i]) {
                last_axes[i] = axes[i];
            }
        }
    }
    
    void update_aux_channels() {
        // AUX1: Flight mode from buttons 0-2
        if (buttons[0]) aux1 = 1000;        // Manual
        else if (buttons[1]) aux1 = 1500;   // Stabilize
        else if (buttons[2]) aux1 = 2000;   // Auto
        else aux1 = 1500;
        
        // AUX2: From buttons 3-4
        if (buttons[3]) aux2 = 2000;
        else if (buttons[4]) aux2 = 1000;
        else aux2 = 1500;
        
        // AUX3: From trigger (axis 2) if available
        if (axes.size() > 2) {
            aux3 = axis_to_pwm(axes[2]);  // Axis 2 as trigger
        } else {
            aux3 = 1500;
        }
        
        // AUX4: From hat switch or button 5
        if (!hats.empty() && hats[0].second == 1) aux4 = 2000;   // Hat up
        else if (!hats.empty() && hats[0].second == -1) aux4 = 1000; // Hat down
        else if (buttons.size() > 5 && buttons[5]) aux4 = 2000;  // Button 5
        else aux4 = 1500;
    }
    
    void print_detailed_status() {
        // Main flight controls
        std::cout << "FLIGHT: R:" << std::setw(4) << roll 
                  << " P:" << std::setw(4) << pitch
                  << " Y:" << std::setw(4) << yaw
                  << " T:" << std::setw(4) << throttle;
        
        // AUX channels
        std::cout << " | AUX: " << aux1;
        
        // Show flight mode name for AUX1
        auto mode_it = flight_modes.find(aux1);
        if (mode_it != flight_modes.end()) {
            std::cout << "(" << mode_it->second << ")";
        }
        
        std::cout << " " << aux2 << " " << aux3 << " " << aux4;
        
        // Active buttons
        std::vector<int> active_buttons;
        for (size_t i = 0; i < buttons.size(); i++) {
            if (buttons[i]) active_buttons.push_back(i);
        }
        
        if (!active_buttons.empty()) {
            std::cout << " | BTNS: ";
            for (size_t i = 0; i < active_buttons.size(); i++) {
                if (i > 0) std::cout << ",";
                std::cout << active_buttons[i];
            }
        }
        
        // Triggers (using axis 2 and 5 if available)
        if (axes.size() > 2) {
            std::cout << " | TRIG: A2=" << normalize_axis(axes[2]);
        }
        if (axes.size() > 5) {
            std::cout << " A5=" << normalize_axis(axes[5]);
        }
        
        // Hats
        for (size_t i = 0; i < hats.size(); i++) {
            if (hats[i].first != 0 || hats[i].second != 0) {
                std::cout << " | HAT" << i << ":" << hats[i].first 
                          << "," << hats[i].second;
            }
        }
        
        // Raw axis values for debugging
        std::cout << " | RAW:";
        for (int i : {0, 1, 3, 4}) {
            if (i < axes.size()) {
                std::cout << " A" << i << ":" << axes[i];
            }
        }
        
        std::cout << std::endl;
    }
    
    void run() {
        std::cout << "Advanced Joystick Reader - Press Ctrl+C to exit" << std::endl;
        std::cout << "FLIGHT: Roll Pitch Yaw Throttle | AUX channels | Buttons | Triggers | Hats" << std::endl;
        std::cout << "----------------------------------------------------------------------------" << std::endl;
        
        struct js_event event;
        
        while (running) {
            ssize_t bytes;
            while ((bytes = read(fd, &event, sizeof(event))) > 0) {
                if (bytes == sizeof(event)) {
                    process_event(event);
                    print_detailed_status();
                }
            }
            usleep(10000);  // 10ms
        }
    }
};

int main() {
    signal(SIGINT, signal_handler);
    
    try {
        AdvancedJoystickReader reader("/dev/input/js0");
        reader.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Try: sudo ./joystick_reader" << std::endl;
        return 1;
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}