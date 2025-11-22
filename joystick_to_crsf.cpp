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

class CRSFGenerator {
public:
    // CRSF Protocol Constants - use #define instead of static constexpr
    #define CRSF_SYNC_BYTE 0xC8
    #define CRSF_FRAME_SIZE 26
    #define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
    
    // Channel mapping
    enum CRSF_CHANNEL_MAP {
        CH_ROLL = 0,
        CH_PITCH = 1,
        CH_THROTTLE = 2,
        CH_YAW = 3,
        CH_AUX1 = 4,
        CH_AUX2 = 5,
        CH_AUX3 = 6,
        CH_AUX4 = 7,
        CH_AUX5 = 8,
        CH_AUX6 = 9,
        CH_AUX7 = 10,
        CH_AUX8 = 11,
        CH_AUX9 = 12,
        CH_AUX10 = 13,
        CH_AUX11 = 14,
        CH_AUX12 = 15
    };
    
public:
    // Convert RC value (1000-2000) to CRSF format (0-1984)
    static uint16_t rc_to_crsf(int rc_value) {
        // Limit to valid range
        if (rc_value < 1000) rc_value = 1000;
        if (rc_value > 2000) rc_value = 2000;
        
        // Convert 1000-2000 to 0-1984
        return static_cast<uint16_t>((rc_value - 1000) * 1.984);
    }
    
    // Generate CRSF RC channels packet
    static std::vector<uint8_t> generate_crsf_frame(const std::vector<uint16_t>& channels) {
        std::vector<uint8_t> frame;
        
        // Frame structure:
        // [SYNC_BYTE][FRAME_SIZE][FRAME_TYPE][PAYLOAD][CRC]
        
        // Payload: 16 channels * 11 bits = 22 bytes
        uint8_t payload[22] = {0};
        
        // Pack 11-bit channels into bytes
        for (int i = 0; i < 16; i++) {
            uint16_t channel_value = (i < channels.size()) ? channels[i] : 992; // Center position
            
            // Pack 11-bit value into bytes
            int byte_index = (i * 11) / 8;
            int bit_index = (i * 11) % 8;
            
            payload[byte_index] |= (channel_value << bit_index) & 0xFF;
            payload[byte_index + 1] |= (channel_value >> (8 - bit_index)) & 0xFF;
            if (bit_index >= 6) { // Needs third byte
                payload[byte_index + 2] |= (channel_value >> (16 - bit_index)) & 0xFF;
            }
        }
        
        // Build frame
        frame.push_back(CRSF_SYNC_BYTE);                    // Sync byte
        frame.push_back(CRSF_FRAME_SIZE);                   // Frame size
        frame.push_back(CRSF_FRAMETYPE_RC_CHANNELS_PACKED); // Frame type
        
        // Add payload
        for (int i = 0; i < 22; i++) {
            frame.push_back(payload[i]);
        }
        
        // Calculate CRC (simple XOR for now - real CRSF uses CRC8)
        uint8_t crc = 0;
        for (size_t i = 2; i < frame.size(); i++) { // Start from frame size byte
            crc ^= frame[i];
        }
        frame.push_back(crc);
        
        return frame;
    }
    
    // Print CRSF frame in hex format
    static void print_crsf_frame(const std::vector<uint8_t>& frame) {
        std::cout << "CRSF Frame: ";
        for (uint8_t byte : frame) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
    }
    
    // Print channel values
    static void print_channels(const std::vector<uint16_t>& channels) {
        const char* channel_names[] = {
            "ROLL", "PITCH", "THROTTLE", "YAW", "AUX1", "AUX2", "AUX3", "AUX4",
            "AUX5", "AUX6", "AUX7", "AUX8", "AUX9", "AUX10", "AUX11", "AUX12"
        };
        
        std::cout << "Channels: ";
        for (int i = 0; i < 8; i++) { // Show first 8 channels
            if (i < channels.size()) {
                // Convert back to RC value for display
                int rc_value = (channels[i] / 1.984) + 1000;
                std::cout << channel_names[i] << ":" << rc_value << " ";
            }
        }
        std::cout << std::endl;
    }
};

class JoystickToCRSF {
private:
    int fd;
    
    // RC values (1000-2000 range)
    int roll, pitch, yaw, throttle;
    int aux1, aux2, aux3, aux4;
    
    // Raw inputs
    std::vector<bool> buttons;
    std::vector<int> axes;
    std::vector<std::pair<int, int>> hats;
    
    // Flight mode mapping
    std::map<int, std::string> flight_modes = {
        {1000, "MANUAL"},
        {1500, "STABILIZE"},
        {2000, "AUTO"}
    };

public:
    JoystickToCRSF(const std::string& device) {
        fd = open(device.c_str(), O_RDONLY);
        if (fd == -1) {
            throw std::runtime_error("Could not open joystick device");
        }
        
        // Initialize all values to center
        roll = pitch = yaw = throttle = 1500;
        aux1 = aux2 = aux3 = aux4 = 1500;
        
        buttons.resize(20, false);
        axes.resize(10, 0);
        hats.resize(2, {0, 0});
        
        // Set non-blocking mode
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        
        std::cout << "Joystick to CRSF Converter Initialized" << std::endl;
        std::cout << "Axis Mapping: " << std::endl;
        std::cout << "  Axis 0: YAW | Axis 1: THROTTLE | Axis 3: ROLL | Axis 4: PITCH" << std::endl;
    }
    
    ~JoystickToCRSF() {
        if (fd != -1) close(fd);
    }
    
    int normalized_to_rc(int normalized_value) {
        // Convert -100 to 100 range to 1000-2000
        int rc_value = 1500 + (normalized_value * 5);
        
        // Limit to valid range
        if (rc_value < 1000) rc_value = 1000;
        if (rc_value > 2000) rc_value = 2000;
        
        return rc_value;
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
        // Convert normalized values to RC values (1000-2000)
        if (axes.size() > 4) {
            int yaw_norm = axes[0] / 327.67;
            int throttle_norm = -axes[1] / 327.67;  // Inverted
            int roll_norm = axes[3] / 327.67;
            int pitch_norm = axes[4] / 327.67;
            
            yaw = normalized_to_rc(yaw_norm);
            throttle = normalized_to_rc(throttle_norm);
            roll = normalized_to_rc(roll_norm);
            pitch = normalized_to_rc(pitch_norm);
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
            int trigger_norm = axes[2] / 327.67;
            aux3 = normalized_to_rc(trigger_norm);
        } else {
            aux3 = 1500;
        }
        
        // AUX4: From hat switch
        if (!hats.empty() && hats[0].second == 1) aux4 = 2000;   // Hat up
        else if (!hats.empty() && hats[0].second == -1) aux4 = 1000; // Hat down
        else aux4 = 1500;
    }
    
    std::vector<uint16_t> get_crsf_channels() {
        std::vector<uint16_t> channels(16, 992); // Initialize with center position
        
        // Map RC values to CRSF channels
        channels[CRSFGenerator::CH_ROLL] = CRSFGenerator::rc_to_crsf(roll);
        channels[CRSFGenerator::CH_PITCH] = CRSFGenerator::rc_to_crsf(pitch);
        channels[CRSFGenerator::CH_THROTTLE] = CRSFGenerator::rc_to_crsf(throttle);
        channels[CRSFGenerator::CH_YAW] = CRSFGenerator::rc_to_crsf(yaw);
        channels[CRSFGenerator::CH_AUX1] = CRSFGenerator::rc_to_crsf(aux1);
        channels[CRSFGenerator::CH_AUX2] = CRSFGenerator::rc_to_crsf(aux2);
        channels[CRSFGenerator::CH_AUX3] = CRSFGenerator::rc_to_crsf(aux3);
        channels[CRSFGenerator::CH_AUX4] = CRSFGenerator::rc_to_crsf(aux4);
        
        return channels;
    }
    
    void print_status() {
        // Main flight controls
        std::cout << "RC: R:" << std::setw(4) << roll 
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
        
        std::cout << std::endl;
    }
    
    void run() {
        std::cout << "Joystick to CRSF Converter - Press Ctrl+C to exit" << std::endl;
        std::cout << "Generating CRSF frames at 50Hz" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
        
        struct js_event event;
        int frame_count = 0;
        
        while (running) {
            // Read joystick events
            ssize_t bytes;
            while ((bytes = read(fd, &event, sizeof(event))) > 0) {
                if (bytes == sizeof(event)) {
                    process_event(event);
                }
            }
            
            // Generate CRSF frame at 50Hz (every 20ms)
            if (frame_count % 2 == 0) { // Simple timing for 50Hz
                // Get current channel values
                auto channels = get_crsf_channels();
                
                // Generate CRSF frame
                auto crsf_frame = CRSFGenerator::generate_crsf_frame(channels);
                
                // Print status and frame
                print_status();
                CRSFGenerator::print_channels(channels);
                CRSFGenerator::print_crsf_frame(crsf_frame);
                std::cout << "---" << std::endl;
            }
            
            frame_count++;
            usleep(10000); // 10ms sleep
        }
    }
};

int main() {
    signal(SIGINT, signal_handler);
    
    try {
        JoystickToCRSF converter("/dev/input/js0");
        converter.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Try: sudo ./joystick_to_crsf" << std::endl;
        return 1;
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}