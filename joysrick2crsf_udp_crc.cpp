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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

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
        
        // Convert 1000-2000 to 0-1984
        return static_cast<uint16_t>((rc_value - 1000) * 1.984);
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
        
        // Calculate CRC exactly like Python receiver: from frame[2] to frame[-1] (type byte to before CRC)
        // Python: crc8_data(frame[2:-1]) - so we need to calculate from index 2 to size-2
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
            
            std::cout << "CRC calculation: data=";
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
                int rc_value = (channels[i] / 1.984) + 1000;
                std::cout << channel_names[i] << ":" << rc_value << " ";
            }
        }
        std::cout << std::endl;
    }
};

class CRSFUDPSender {
private:
    int sockfd;
    struct sockaddr_in server_addr;
    std::string remote_host;
    int remote_port;
    bool connected;

public:
    CRSFUDPSender(const std::string& host = "127.0.0.1", int port = 8888) 
        : remote_host(host), remote_port(port), connected(false) {
        
        // Create UDP socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            throw std::runtime_error("Could not create UDP socket");
        }
        
        // Set socket to non-blocking
        int flags = fcntl(sockfd, F_GETFL, 0);
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
        
        // Configure server address
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(remote_port);
        
        if (inet_pton(AF_INET, remote_host.c_str(), &server_addr.sin_addr) <= 0) {
            close(sockfd);
            throw std::runtime_error("Invalid address: " + remote_host);
        }
        
        connected = true;
        std::cout << "UDP Sender initialized: " << remote_host << ":" << remote_port << std::endl;
    }
    
    ~CRSFUDPSender() {
        if (sockfd != -1) {
            close(sockfd);
        }
    }
    
    bool send_frame_with_header(const std::vector<uint8_t>& frame) {
        if (!connected) return false;
        
        // Create packet with header: [4-byte size][frame data]
        std::vector<uint8_t> packet;
        uint32_t frame_size = frame.size();
        
        // Add size header (big-endian)
        packet.push_back((frame_size >> 24) & 0xFF);
        packet.push_back((frame_size >> 16) & 0xFF);
        packet.push_back((frame_size >> 8) & 0xFF);
        packet.push_back(frame_size & 0xFF);
        
        // Add frame data
        packet.insert(packet.end(), frame.begin(), frame.end());
        
        ssize_t bytes_sent = sendto(sockfd, packet.data(), packet.size(), 0,
                                   (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        if (bytes_sent == -1) {
            // Non-blocking socket might return EAGAIN/EWOULDBLOCK
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return false; // Buffer full, try again later
            }
            return false;
        }
        
        return (bytes_sent == static_cast<ssize_t>(packet.size()));
    }
    
    std::string get_status() const {
        return "UDP->" + remote_host + ":" + std::to_string(remote_port);
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
    
    // UDP Sender
    CRSFUDPSender udp_sender;
    
    // Statistics
    int frames_sent;
    int frames_failed;

public:
    JoystickToCRSF(const std::string& device, const std::string& udp_host = "127.0.0.1", int udp_port = 8888) 
        : udp_sender(udp_host, udp_port), frames_sent(0), frames_failed(0) {
        
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
        
        std::cout << "Joystick to CRSF Converter with UDP Initialized" << std::endl;
        std::cout << "UDP Target: " << udp_sender.get_status() << std::endl;
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
        std::cout << " | AUX: " << aux1 << " " << aux2 << " " << aux3 << " " << aux4;
        
        // UDP statistics
        std::cout << " | UDP: " << frames_sent << " sent, " << frames_failed << " failed";
        
        std::cout << std::endl;
    }
    
    void run() {
        std::cout << "Joystick to CRSF UDP Converter - Press Ctrl+C to exit" << std::endl;
        std::cout << "Sending CRSF frames via UDP at 50Hz" << std::endl;
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
            if (frame_count % 2 == 0) {
                // Get current channel values
                auto channels = get_crsf_channels();
                
                // Generate CRSF frame with proper CRC
                auto crsf_frame = CRSFGenerator::generate_crsf_frame(channels);
                
                // Print to console
                print_status();
                CRSFGenerator::print_channels(channels);
                CRSFGenerator::print_crsf_frame(crsf_frame);
                
                // Send via UDP (with header for reliable parsing)
                if (udp_sender.send_frame_with_header(crsf_frame)) {
                    std::cout << "✓ UDP sent successfully" << std::endl;
                    frames_sent++;
                } else {
                    std::cout << "✗ UDP send failed" << std::endl;
                    frames_failed++;
                }
                std::cout << "---" << std::endl;
            }
            
            frame_count++;
            usleep(10000); // 10ms sleep
        }
    }
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    
    // Parse command line arguments for UDP configuration
    std::string udp_host = "127.0.0.1";
    int udp_port = 8888;
    
    if (argc >= 2) {
        udp_host = argv[1];
    }
    if (argc >= 3) {
        udp_port = std::stoi(argv[2]);
    }
    
    std::cout << "Starting with UDP configuration: " << udp_host << ":" << udp_port << std::endl;
    
    try {
        JoystickToCRSF converter("/dev/input/js0", udp_host, udp_port);
        converter.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Usage: " << argv[0] << " [host] [port]" << std::endl;
        std::cerr << "Default: 127.0.0.1:8888" << std::endl;
        return 1;
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}