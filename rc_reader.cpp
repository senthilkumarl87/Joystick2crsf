#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <map>
#include <csignal>  // Add this for signal handling
#include <sys/select.h>  // Add this for select()

// MAVLink message IDs for RC data
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36
#define MAVLINK_MSG_ID_HEARTBEAT 0

#define MAVLINK_MAX_PACKET_LEN 263

// MAVLink message structures for RC data
typedef struct __mavlink_rc_channels_t {
    uint32_t time_boot_ms;
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint16_t chan9_raw;
    uint16_t chan10_raw;
    uint16_t chan11_raw;
    uint16_t chan12_raw;
    uint16_t chan13_raw;
    uint16_t chan14_raw;
    uint16_t chan15_raw;
    uint16_t chan16_raw;
    uint16_t chan17_raw;
    uint16_t chan18_raw;
    uint8_t chancount;
    uint8_t rssi;
} mavlink_rc_channels_t;

typedef struct __mavlink_rc_channels_raw_t {
    uint32_t time_boot_ms;
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint8_t port;
    uint8_t rssi;
} mavlink_rc_channels_raw_t;

typedef struct __mavlink_servo_output_raw_t {
    uint32_t time_usec;
    uint16_t servo1_raw;
    uint16_t servo2_raw;
    uint16_t servo3_raw;
    uint16_t servo4_raw;
    uint16_t servo5_raw;
    uint16_t servo6_raw;
    uint16_t servo7_raw;
    uint16_t servo8_raw;
    uint8_t port;
} mavlink_servo_output_raw_t;

typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

// MAVLink message structure
typedef struct __mavlink_message {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint8_t msgid;
    uint8_t payload[MAVLINK_MAX_PACKET_LEN];
    uint8_t crc_extra;
} mavlink_message_t;

// MAVLink parser class
class MavlinkParser {
private:
    enum ParseState {
        WAITING_START,
        GOT_START,
        GOT_LENGTH,
        GOT_SEQ,
        GOT_SYSID,
        GOT_COMPID,
        GOT_MSGID,
        READING_PAYLOAD,
        GOT_CRC1,
        GOT_CRC2
    };
    
    ParseState state;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t payload_index;
    uint8_t expected_length;
    mavlink_message_t current_msg;
    
public:
    MavlinkParser() : state(WAITING_START), payload_index(0), expected_length(0) {
        memset(&current_msg, 0, sizeof(current_msg));
    }
    
    bool parse_char(uint8_t c, mavlink_message_t& msg) {
        switch (state) {
            case WAITING_START:
                if (c == 0xFE) { // MAVLink v1.0 start byte
                    state = GOT_START;
                }
                break;
                
            case GOT_START:
                current_msg.len = c;
                expected_length = c;
                state = GOT_LENGTH;
                break;
                
            case GOT_LENGTH:
                current_msg.seq = c;
                state = GOT_SEQ;
                break;
                
            case GOT_SEQ:
                current_msg.sysid = c;
                state = GOT_SYSID;
                break;
                
            case GOT_SYSID:
                current_msg.compid = c;
                state = GOT_COMPID;
                break;
                
            case GOT_COMPID:
                current_msg.msgid = c;
                payload_index = 0;
                if (expected_length > 0) {
                    state = READING_PAYLOAD;
                } else {
                    state = GOT_CRC1;
                }
                break;
                
            case READING_PAYLOAD:
                if (payload_index < expected_length) {
                    current_msg.payload[payload_index++] = c;
                }
                if (payload_index >= expected_length) {
                    state = GOT_CRC1;
                }
                break;
                
            case GOT_CRC1:
                // Skip CRC verification for simplicity
                state = GOT_CRC2;
                break;
                
            case GOT_CRC2:
                // Message complete
                msg = current_msg;
                state = WAITING_START;
                return true;
        }
        
        return false;
    }
};

class RCDataExtractor {
private:
    int uart_fd;
    std::atomic<bool> running;
    std::thread read_thread;
    MavlinkParser parser;
    
    // Logging
    std::ofstream logfile;
    std::string log_filename;
    bool enable_logging;
    
    // RC Data storage
    std::map<int, uint16_t> current_channels;
    uint8_t rssi;
    uint32_t last_update_ms;
    uint8_t channel_count;

public:
    RCDataExtractor(bool logging = true) 
        : uart_fd(-1), running(false), enable_logging(logging), rssi(0), last_update_ms(0), channel_count(0) {
        
        if (enable_logging) {
            setupLogFile();
        }
    }
    
    ~RCDataExtractor() {
        stop();
        if (logfile.is_open()) {
            logfile.close();
        }
    }

private:
    void setupLogFile() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);
        
        std::stringstream timestamp;
        timestamp << std::put_time(&tm, "%Y%m%d_%H%M%S");
        
        log_filename = "rc_data_" + timestamp.str() + ".csv";
        logfile.open(log_filename, std::ios::out | std::ios::app);
        
        if (logfile.is_open()) {
            std::cout << "RC Data logging to: " << log_filename << std::endl;
            // Write CSV header
            logfile << "Timestamp,MessageType,ChannelCount,RSSI";
            for (int i = 1; i <= 18; i++) {
                logfile << ",Ch" << i;
            }
            logfile << std::endl;
            logfile.flush();
        } else {
            std::cerr << "Warning: Could not open log file: " << log_filename << std::endl;
            enable_logging = false;
        }
    }

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::tm tm = *std::localtime(&time_t);
        
        std::stringstream ss;
        ss << std::put_time(&tm, "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    void logRCData(const std::string& message_type) {
        if (enable_logging && logfile.is_open()) {
            logfile << getCurrentTimestamp() << "," << message_type << "," 
                    << static_cast<int>(channel_count) << "," << static_cast<int>(rssi);
            
            for (int i = 1; i <= 18; i++) {
                if (current_channels.find(i) != current_channels.end()) {
                    logfile << "," << current_channels[i];
                } else {
                    logfile << ","; // Empty if channel not available
                }
            }
            logfile << std::endl;
            logfile.flush();
        }
    }

    void displayRCData(const std::string& message_type) {
        std::cout << "\r[" << getCurrentTimestamp() << "] " << message_type 
                  << " - Channels: " << static_cast<int>(channel_count) 
                  << ", RSSI: " << static_cast<int>(rssi) << "%";
        
        // Display first 8 channels for compact view
        for (int i = 1; i <= 8; i++) {
            if (current_channels.find(i) != current_channels.end()) {
                std::cout << " Ch" << i << ":" << current_channels[i];
            }
        }
        std::cout << "     " << std::flush; // Clear line
    }

public:
    bool initializeUART(const std::string& port, int baudrate = 115200) {
        uart_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd < 0) {
            std::cerr << "Error opening UART port: " << port << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(uart_fd, &tty) != 0) {
            std::cerr << "Error getting UART attributes" << std::endl;
            close(uart_fd);
            return false;
        }

        // Set baud rate
        speed_t speed;
        switch (baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B115200; break;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 configuration
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= (CS8 | CREAD | CLOCAL);
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting UART attributes" << std::endl;
            close(uart_fd);
            return false;
        }

        std::cout << "UART port " << port << " initialized at " << baudrate << " baud" << std::endl;
        return true;
    }

    void start() {
        running = true;
        read_thread = std::thread(&RCDataExtractor::readLoop, this);
    }

    void stop() {
        running = false;
        if (read_thread.joinable()) {
            read_thread.join();
        }
        if (uart_fd >= 0) {
            close(uart_fd);
            uart_fd = -1;
        }
        
        if (enable_logging && logfile.is_open()) {
            logfile.close();
        }
        
        std::cout << "\nRC Data extractor stopped" << std::endl;
    }

    void printCurrentChannels() {
        std::cout << "\n\n=== CURRENT RC CHANNELS ===" << std::endl;
        std::cout << "Channel Count: " << static_cast<int>(channel_count) << std::endl;
        std::cout << "RSSI: " << static_cast<int>(rssi) << "%" << std::endl;
        std::cout << "Channels:" << std::endl;
        
        for (const auto& channel : current_channels) {
            std::cout << "  Ch" << channel.first << ": " << channel.second;
            // Interpret common channels
            switch (channel.first) {
                case 1: std::cout << " (Roll)"; break;
                case 2: std::cout << " (Pitch)"; break;
                case 3: std::cout << " (Throttle)"; break;
                case 4: std::cout << " (Yaw)"; break;
                case 5: std::cout << " (Mode/Flight Mode)"; break;
                case 6: std::cout << " (Aux1)"; break;
                case 7: std::cout << " (Aux2)"; break;
                case 8: std::cout << " (Aux3)"; break;
            }
            std::cout << std::endl;
        }
        std::cout << "=============================" << std::endl;
    }

private:
    void readLoop() {
        mavlink_message_t msg;
        uint8_t buffer[256];
        
        std::cout << "Starting RC data extraction..." << std::endl;
        std::cout << "Waiting for RC data from Pixhawk..." << std::endl;

        while (running) {
            ssize_t n = read(uart_fd, buffer, sizeof(buffer));
            
            if (n > 0) {
                for (ssize_t i = 0; i < n; i++) {
                    if (parser.parse_char(buffer[i], msg)) {
                        handleMavlinkMessage(msg);
                    }
                }
            } else if (n < 0) {
                std::cerr << "Error reading from UART" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void handleMavlinkMessage(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_RC_CHANNELS:
                handleRCChannels(msg);
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                handleRCChannelsRaw(msg);
                break;
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                handleServoOutputRaw(msg);
                break;
            case MAVLINK_MSG_ID_HEARTBEAT:
                // Just to know we're connected
                static bool first_heartbeat = true;
                if (first_heartbeat) {
                    std::cout << "\nConnected to Pixhawk! Waiting for RC data..." << std::endl;
                    first_heartbeat = false;
                }
                break;
        }
    }

    void handleRCChannels(const mavlink_message_t& msg) {
        mavlink_rc_channels_t rc_channels;
        memcpy(&rc_channels, msg.payload, sizeof(rc_channels));
        
        // Update channel data
        current_channels.clear();
        if (rc_channels.chan1_raw != UINT16_MAX) current_channels[1] = rc_channels.chan1_raw;
        if (rc_channels.chan2_raw != UINT16_MAX) current_channels[2] = rc_channels.chan2_raw;
        if (rc_channels.chan3_raw != UINT16_MAX) current_channels[3] = rc_channels.chan3_raw;
        if (rc_channels.chan4_raw != UINT16_MAX) current_channels[4] = rc_channels.chan4_raw;
        if (rc_channels.chan5_raw != UINT16_MAX) current_channels[5] = rc_channels.chan5_raw;
        if (rc_channels.chan6_raw != UINT16_MAX) current_channels[6] = rc_channels.chan6_raw;
        if (rc_channels.chan7_raw != UINT16_MAX) current_channels[7] = rc_channels.chan7_raw;
        if (rc_channels.chan8_raw != UINT16_MAX) current_channels[8] = rc_channels.chan8_raw;
        if (rc_channels.chan9_raw != UINT16_MAX) current_channels[9] = rc_channels.chan9_raw;
        if (rc_channels.chan10_raw != UINT16_MAX) current_channels[10] = rc_channels.chan10_raw;
        if (rc_channels.chan11_raw != UINT16_MAX) current_channels[11] = rc_channels.chan11_raw;
        if (rc_channels.chan12_raw != UINT16_MAX) current_channels[12] = rc_channels.chan12_raw;
        if (rc_channels.chan13_raw != UINT16_MAX) current_channels[13] = rc_channels.chan13_raw;
        if (rc_channels.chan14_raw != UINT16_MAX) current_channels[14] = rc_channels.chan14_raw;
        if (rc_channels.chan15_raw != UINT16_MAX) current_channels[15] = rc_channels.chan15_raw;
        if (rc_channels.chan16_raw != UINT16_MAX) current_channels[16] = rc_channels.chan16_raw;
        
        channel_count = rc_channels.chancount;
        rssi = rc_channels.rssi;
        last_update_ms = rc_channels.time_boot_ms;
        
        displayRCData("RC_CHANNELS");
        logRCData("RC_CHANNELS");
    }

    void handleRCChannelsRaw(const mavlink_message_t& msg) {
        mavlink_rc_channels_raw_t rc_raw;
        memcpy(&rc_raw, msg.payload, sizeof(rc_raw));
        
        // Update channel data
        current_channels.clear();
        if (rc_raw.chan1_raw != 0) current_channels[1] = rc_raw.chan1_raw;
        if (rc_raw.chan2_raw != 0) current_channels[2] = rc_raw.chan2_raw;
        if (rc_raw.chan3_raw != 0) current_channels[3] = rc_raw.chan3_raw;
        if (rc_raw.chan4_raw != 0) current_channels[4] = rc_raw.chan4_raw;
        if (rc_raw.chan5_raw != 0) current_channels[5] = rc_raw.chan5_raw;
        if (rc_raw.chan6_raw != 0) current_channels[6] = rc_raw.chan6_raw;
        if (rc_raw.chan7_raw != 0) current_channels[7] = rc_raw.chan7_raw;
        if (rc_raw.chan8_raw != 0) current_channels[8] = rc_raw.chan8_raw;
        
        channel_count = 8; // RC_CHANNELS_RAW typically has 8 channels
        rssi = rc_raw.rssi;
        last_update_ms = rc_raw.time_boot_ms;
        
        displayRCData("RC_CHANNELS_RAW");
        logRCData("RC_CHANNELS_RAW");
    }

    void handleServoOutputRaw(const mavlink_message_t& msg) {
        mavlink_servo_output_raw_t servo_out;
        memcpy(&servo_out, msg.payload, sizeof(servo_out));
        
        // Servo outputs (what the flight controller sends to servos/ESCs)
        std::cout << "\n[SERVO_OUTPUT] ";
        for (int i = 1; i <= 8; i++) {
            uint16_t value = 0;
            switch (i) {
                case 1: value = servo_out.servo1_raw; break;
                case 2: value = servo_out.servo2_raw; break;
                case 3: value = servo_out.servo3_raw; break;
                case 4: value = servo_out.servo4_raw; break;
                case 5: value = servo_out.servo5_raw; break;
                case 6: value = servo_out.servo6_raw; break;
                case 7: value = servo_out.servo7_raw; break;
                case 8: value = servo_out.servo8_raw; break;
            }
            if (value != 0) {
                std::cout << "S" << i << ":" << value << " ";
            }
        }
        std::cout << "     " << std::flush;
    }
};

// Signal handler for graceful shutdown
static std::atomic<bool> shutdown_requested(false);
static void signal_handler(int signal) {
    shutdown_requested = true;
}

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyACM0"; // Pixhawk typically appears as ACM0
    int baudrate = 57600; // Common Pixhawk baud rate
    bool enable_logging = true;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baudrate = std::stoi(argv[++i]);
        } else if (arg == "--no-log") {
            enable_logging = false;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--port DEVICE] [--baud RATE] [--no-log]" << std::endl;
            std::cout << "Default: " << port << " at " << baudrate << " baud" << std::endl;
            return 0;
        }
    }
    
    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    
    RCDataExtractor extractor(enable_logging);
    
    std::cout << "Pixhawk RC Data Extractor" << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Baud rate: " << baudrate << std::endl;
    std::cout << "Logging: " << (enable_logging ? "enabled" : "disabled") << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "Press 'p' to print current channel values" << std::endl;
    std::cout << std::endl;
    
    if (!extractor.initializeUART(port, baudrate)) {
        std::cerr << "Failed to initialize UART port" << std::endl;
        return -1;
    }
    
    extractor.start();
    
    // Main loop with keyboard input handling
    while (!shutdown_requested) {
        // Check for keyboard input
        struct timeval tv = {0, 100000}; // 100ms timeout
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        
        int ready = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        
        if (ready > 0) {
            char c;
            read(STDIN_FILENO, &c, 1);
            if (c == 'p' || c == 'P') {
                extractor.printCurrentChannels();
            } else if (c == 'q' || c == 'Q') {
                shutdown_requested = true;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    extractor.stop();
    std::cout << "\nShutdown complete." << std::endl;
    
    return 0;
}