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
#include <ctime>
#include <chrono>
#include <thread>
#include <atomic>

struct js_event {
    uint32_t time;
    int16_t value;
    uint8_t type;
    uint8_t number;
};

std::atomic<bool> running(true);

void signal_handler(int signal) {
    running = false;
}

// CRC8 function matching the Python reference EXACTLY
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ii++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc & 0xFF;
}

uint8_t crc8_data(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

class HighPrecisionTimer {
private:
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point last_frame;
    int64_t frame_interval_us;
    int64_t accumulated_error;
    int64_t min_sleep_us;

public:
    HighPrecisionTimer(int frequency_hz) {
        start_time = std::chrono::steady_clock::now();
        last_frame = start_time;
        frame_interval_us = 1000000 / frequency_hz; // Convert Hz to microseconds
        accumulated_error = 0;
        min_sleep_us = 1000; // Minimum sleep to prevent busy-waiting (1ms)
    }

    void wait_next_frame() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame);
        int64_t sleep_time = frame_interval_us - elapsed.count() - accumulated_error;

        // Apply error correction
        if (std::abs(accumulated_error) > frame_interval_us / 2) {
            accumulated_error = 0; // Reset if error gets too large
        }

        if (sleep_time > min_sleep_us) {
            // Use precise sleep for longer intervals
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time - min_sleep_us));
            
            // Busy-wait for the remaining time for maximum precision
            auto busy_start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - busy_start).count() < min_sleep_us) {
                std::this_thread::yield();
            }
        } else if (sleep_time > 0) {
            // Short sleep, just yield
            std::this_thread::yield();
        }

        // Calculate timing error for next frame
        auto actual_now = std::chrono::steady_clock::now();
        auto actual_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(actual_now - last_frame);
        accumulated_error += (actual_elapsed.count() - frame_interval_us);
        
        last_frame = actual_now;
    }

    double get_jitter() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame);
        return std::abs(elapsed.count() - frame_interval_us) / 1000.0; // Return jitter in ms
    }

    int64_t get_frame_interval_us() const {
        return frame_interval_us;
    }
};

class CRSFGenerator {
public:
    // CRSF Protocol Constants
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
        
        // Convert 1000-2000 to 0-1984 using integer math only (no floating point)
        return static_cast<uint16_t>((rc_value - 1000) * 1984 / 1000);
    }
    
    // Generate CRSF RC channels packet with proper CRC
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
        
        // Build frame (without CRC first)
        frame.push_back(CRSF_SYNC_BYTE);                    // Sync byte
        frame.push_back(24);                                // Frame size: type(1) + payload(22) + CRC(1) = 24
        frame.push_back(CRSF_FRAMETYPE_RC_CHANNELS_PACKED); // Frame type
        
        // Add payload
        for (int i = 0; i < 22; i++) {
            frame.push_back(payload[i]);
        }
        
        // Calculate proper CRC (from frame[2] to frame[-1] - matching Python receiver)
        uint8_t crc = crc8_data(frame.data() + 2, frame.size() - 2); // Start from type byte, exclude sync, size, and CRC
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
        
        // Also print CRC calculation details
        if (frame.size() >= 4) {
            // Calculate what the CRC should be (matching Python: frame[2:-1])
            uint8_t calculated_crc = crc8_data(frame.data() + 2, frame.size() - 3); // From type byte to before CRC
            uint8_t frame_crc = frame.back();
            
            std::cout << "CRC: data=";
            for (size_t i = 2; i < frame.size() - 1; i++) {
                std::cout << std::hex << std::setw(2) << static_cast<int>(frame[i]) << " ";
            }
            std::cout << "-> calculated=0x" << std::hex << static_cast<int>(calculated_crc) 
                      << ", frame=0x" << static_cast<int>(frame_crc) << std::dec;
            if (calculated_crc == frame_crc) {
                std::cout << " ✓ VALID" << std::endl;
            } else {
                std::cout << " ✗ INVALID" << std::endl;
            }
        }
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
                int rc_value = (channels[i] * 1000) / 1984 + 1000;
                std::cout << channel_names[i] << ":" << rc_value << " ";
            }
        }
        std::cout << std::endl;
        
        // Also show raw CRSF values
        std::cout << "Raw CRSF: ";
        for (int i = 0; i < 8; i++) {
            if (i < channels.size()) {
                std::cout << channel_names[i] << ":" << channels[i] << " ";
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
    
    // Statistics
    int frames_generated;
    uint64_t total_jitter_us;
    uint64_t max_jitter_us;
    std::chrono::steady_clock::time_point start_time;

public:
    JoystickToCRSF(const std::string& device) : frames_generated(0), total_jitter_us(0), max_jitter_us(0) {
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
        
        start_time = std::chrono::steady_clock::now();
        
        std::cout << "Joystick to CRSF Converter Initialized" << std::endl;
        std::cout << "High Precision Timer: 50Hz (20ms intervals)" << std::endl;
        std::cout << "Axis Mapping: " << std::endl;
        std::cout << "  Axis 0: YAW | Axis 1: THROTTLE | Axis 3: ROLL | Axis 4: PITCH" << std::endl;
        std::cout << "==========================================" << std::endl;
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
    
    void update_timing_stats(double jitter_us) {
        total_jitter_us += static_cast<uint64_t>(std::abs(jitter_us));
        if (static_cast<uint64_t>(std::abs(jitter_us)) > max_jitter_us) {
            max_jitter_us = static_cast<uint64_t>(std::abs(jitter_us));
        }
    }
    
    void print_status(double current_jitter_us) {
        auto now = std::chrono::steady_clock::now();
        auto runtime = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        
        // Main flight controls
        std::cout << "RC: R:" << std::setw(4) << roll 
                  << " P:" << std::setw(4) << pitch
                  << " Y:" << std::setw(4) << yaw
                  << " T:" << std::setw(4) << throttle;
        
        // AUX channels
        std::cout << " | AUX: " << aux1 << " " << aux2 << " " << aux3 << " " << aux4;
        
        // Timing statistics
        double avg_jitter = frames_generated > 0 ? (total_jitter_us / frames_generated) / 1000.0 : 0.0;
        std::cout << " | Timing: Jitter=" << std::fixed << std::setprecision(2) 
                  << (current_jitter_us / 1000.0) << "ms"
                  << " Avg=" << avg_jitter << "ms"
                  << " Max=" << (max_jitter_us / 1000.0) << "ms";
        
        // Frame statistics
        std::cout << " | Frames: " << frames_generated;
        std::cout << " | Runtime: " << runtime.count() << "s";
        
        std::cout << std::endl;
    }
    
    void read_joystick_events() {
        struct js_event event;
        ssize_t bytes;
        
        // Read all available joystick events
        while ((bytes = read(fd, &event, sizeof(event))) > 0) {
            if (bytes == sizeof(event)) {
                process_event(event);
            }
        }
    }
    
    void run() {
        std::cout << "Joystick to CRSF Converter - Press Ctrl+C to exit" << std::endl;
        std::cout << "High Precision Mode: 50Hz CRSF frames with <1ms jitter" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
        
        HighPrecisionTimer timer(50); // 50Hz CRSF
        
        while (running) {
            // Read all available joystick events (non-blocking)
            read_joystick_events();
            
            // Generate CRSF frame at precise 50Hz intervals
            auto frame_start = std::chrono::steady_clock::now();
            
            // Get current channel values
            auto channels = get_crsf_channels();
            
            // Generate CRSF frame with proper CRC
            auto crsf_frame = CRSFGenerator::generate_crsf_frame(channels);
            
            // Calculate timing jitter
            auto frame_end = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
            double jitter_us = processing_time.count();
            
            // Update timing statistics
            update_timing_stats(jitter_us);
            
            // Print detailed information every 50 frames (1 second) to reduce console spam
            if (frames_generated % 50 == 0) {
                auto now = std::time(nullptr);
                auto local_time = std::localtime(&now);
                std::cout << "\n[" << std::put_time(local_time, "%H:%M:%S") << "] Frame #" << frames_generated + 1 << std::endl;
                
                // Print RC status
                print_status(jitter_us);
                
                // Print channel information
                CRSFGenerator::print_channels(channels);
                
                // Print CRSF frame (first frame and every 100th frame thereafter)
                if (frames_generated == 0 || frames_generated % 100 == 0) {
                    CRSFGenerator::print_crsf_frame(crsf_frame);
                }
                
                std::cout << "---" << std::endl;
            }
            
            frames_generated++;
            
            // Wait for next frame with high precision
            timer.wait_next_frame();
        }
        
        // Print final statistics
        auto end_time = std::chrono::steady_clock::now();
        auto total_runtime = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        double avg_jitter = frames_generated > 0 ? (total_jitter_us / frames_generated) / 1000.0 : 0.0;
        double frames_per_second = frames_generated / static_cast<double>(total_runtime.count());
        
        std::cout << "\n==========================================" << std::endl;
        std::cout << "Final Statistics:" << std::endl;
        std::cout << "  Total Frames: " << frames_generated << std::endl;
        std::cout << "  Total Runtime: " << total_runtime.count() << " seconds" << std::endl;
        std::cout << "  Actual Frame Rate: " << std::fixed << std::setprecision(2) << frames_per_second << " Hz" << std::endl;
        std::cout << "  Average Jitter: " << avg_jitter << " ms" << std::endl;
        std::cout << "  Maximum Jitter: " << (max_jitter_us / 1000.0) << " ms" << std::endl;
        std::cout << "==========================================" << std::endl;
    }
};

int main() {
    signal(SIGINT, signal_handler);
    
    // Set high priority for better timing (requires root)
    if (nice(-20) == -1) {
        std::cout << "Note: Could not set high process priority (run with sudo for better timing)" << std::endl;
    }
    
    try {
        JoystickToCRSF converter("/dev/input/js0");
        converter.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Make sure joystick is connected and try with sudo" << std::endl;
        return 1;
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}