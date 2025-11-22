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

// Minimal MAVLink definitions (same as before)
#define MAVLINK_MAX_PACKET_LEN 263
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_BATTERY_STATUS 147

// MAVLink data structures (same as before)
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

typedef struct __mavlink_sys_status_t {
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint16_t load;
    uint16_t voltage_battery;
    int16_t current_battery;
    uint8_t battery_remaining;
} mavlink_sys_status_t;

typedef struct __mavlink_gps_raw_int_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
} mavlink_gps_raw_int_t;

typedef struct __mavlink_attitude_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} mavlink_attitude_t;

typedef struct __mavlink_param_value_t {
    char param_id[16];
    float param_value;
    uint16_t param_count;
    uint16_t param_index;
    uint8_t param_type;
} mavlink_param_value_t;

typedef struct __mavlink_battery_status_t {
    int32_t current_consumed;
    int32_t energy_consumed;
    int16_t temperature;
    uint16_t voltages[10];
    int16_t current_battery;
    uint8_t id;
    uint8_t battery_function;
    uint8_t type;
    int8_t battery_remaining;
} mavlink_battery_status_t;

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

// MAVLink parser class (same as before)
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
                // For simplicity, skip CRC verification
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

class MavlinkUARTReader {
private:
    int uart_fd;
    std::atomic<bool> running;
    std::thread read_thread;
    MavlinkParser parser;
    
    // Logging members
    std::ofstream logfile;
    std::ofstream raw_logfile;
    std::string log_filename;
    std::string raw_log_filename;
    bool enable_logging;
    bool enable_raw_logging;

public:
    MavlinkUARTReader(bool logging = true, bool raw_logging = false) 
        : uart_fd(-1), running(false), enable_logging(logging), enable_raw_logging(raw_logging) {
        
        if (enable_logging || enable_raw_logging) {
            setupLogFiles();
        }
    }
    
    ~MavlinkUARTReader() {
        stop();
        if (logfile.is_open()) {
            logfile.close();
        }
        if (raw_logfile.is_open()) {
            raw_logfile.close();
        }
    }

private:
    void setupLogFiles() {
        // Generate timestamp for unique filenames
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);
        
        std::stringstream timestamp;
        timestamp << std::put_time(&tm, "%Y%m%d_%H%M%S");
        
        if (enable_logging) {
            log_filename = "mavlink_log_" + timestamp.str() + ".txt";
            logfile.open(log_filename, std::ios::out | std::ios::app);
            if (!logfile.is_open()) {
                std::cerr << "Warning: Could not open log file: " << log_filename << std::endl;
                enable_logging = false;
            } else {
                std::cout << "Logging to: " << log_filename << std::endl;
                logfile << "MAVLink Log Started at: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << std::endl;
                logfile << "==========================================" << std::endl;
            }
        }
        
        if (enable_raw_logging) {
            raw_log_filename = "mavlink_raw_" + timestamp.str() + ".bin";
            raw_logfile.open(raw_log_filename, std::ios::out | std::ios::binary | std::ios::app);
            if (!raw_logfile.is_open()) {
                std::cerr << "Warning: Could not open raw log file: " << raw_log_filename << std::endl;
                enable_raw_logging = false;
            } else {
                std::cout << "Raw logging to: " << raw_log_filename << std::endl;
            }
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

    void logMessage(const std::string& message) {
        std::string timestamped_msg = "[" + getCurrentTimestamp() + "] " + message;
        
        // Print to console
        std::cout << timestamped_msg << std::endl;
        
        // Write to log file
        if (enable_logging && logfile.is_open()) {
            logfile << timestamped_msg << std::endl;
            logfile.flush(); // Ensure data is written immediately
        }
    }

    void logRawData(const uint8_t* data, size_t length) {
        if (enable_raw_logging && raw_logfile.is_open()) {
            // Add timestamp to raw data
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
            
            raw_logfile.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));
            raw_logfile.write(reinterpret_cast<const char*>(data), length);
            raw_logfile.flush();
        }
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
            case 460800: speed = B460800; break;
            case 500000: speed = B500000; break;
            case 921600: speed = B921600; break;
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
        tty.c_cc[VMIN] = 1;  // Block until at least 1 character received
        tty.c_cc[VTIME] = 0; // No timeout

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting UART attributes" << std::endl;
            close(uart_fd);
            return false;
        }

        logMessage("UART port " + port + " initialized at " + std::to_string(baudrate) + " baud");
        return true;
    }

    void start() {
        running = true;
        read_thread = std::thread(&MavlinkUARTReader::readLoop, this);
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
            logfile << "==========================================" << std::endl;
            logfile << "MAVLink Log Ended" << std::endl;
            logfile.close();
        }
        
        if (enable_raw_logging && raw_logfile.is_open()) {
            raw_logfile.close();
        }
        
        logMessage("MAVLink reader stopped");
    }

private:
    void readLoop() {
        mavlink_message_t msg;
        uint8_t buffer[256];
        
        logMessage("Starting MAVLink message reading loop...");

        while (running) {
            ssize_t n = read(uart_fd, buffer, sizeof(buffer));
            
            if (n > 0) {
                // Log raw data if enabled
                if (enable_raw_logging) {
                    logRawData(buffer, n);
                }
                
                for (ssize_t i = 0; i < n; i++) {
                    if (parser.parse_char(buffer[i], msg)) {
                        handleMavlinkMessage(msg);
                    }
                }
            } else if (n < 0) {
                logMessage("Error reading from UART");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void handleMavlinkMessage(const mavlink_message_t& msg) {
        std::stringstream ss;
        ss << "MAVLink Message - SysID: " << static_cast<int>(msg.sysid)
           << ", CompID: " << static_cast<int>(msg.compid)
           << ", MsgID: " << static_cast<int>(msg.msgid)
           << ", Len: " << static_cast<int>(msg.len);
        logMessage(ss.str());

        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handleHeartbeat(msg);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                handleSysStatus(msg);
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                handleGPSRawInt(msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                handleAttitude(msg);
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE:
                handleParamValue(msg);
                break;
            case MAVLINK_MSG_ID_BATTERY_STATUS:
                handleBatteryStatus(msg);
                break;
            default:
                handleUnknownMessage(msg);
                break;
        }
    }

    void handleHeartbeat(const mavlink_message_t& msg) {
        mavlink_heartbeat_t heartbeat;
        memcpy(&heartbeat, msg.payload, sizeof(heartbeat));
        
        std::stringstream ss;
        ss << "HEARTBEAT - Type: " << static_cast<int>(heartbeat.type)
           << ", Autopilot: " << static_cast<int>(heartbeat.autopilot)
           << ", Mode: " << heartbeat.custom_mode
           << ", Status: " << static_cast<int>(heartbeat.system_status);
        logMessage(ss.str());
    }

    void handleSysStatus(const mavlink_message_t& msg) {
        mavlink_sys_status_t sys_status;
        memcpy(&sys_status, msg.payload, sizeof(sys_status));
        
        std::stringstream ss;
        ss << "SYS_STATUS - Voltage: " << std::fixed << std::setprecision(2) 
           << (sys_status.voltage_battery / 1000.0f) << "V"
           << ", Current: " << (sys_status.current_battery / 100.0f) << "A"
           << ", Battery: " << static_cast<int>(sys_status.battery_remaining) << "%";
        logMessage(ss.str());
    }

    void handleGPSRawInt(const mavlink_message_t& msg) {
        mavlink_gps_raw_int_t gps;
        memcpy(&gps, msg.payload, sizeof(gps));
        
        std::stringstream ss;
        ss << "GPS - Lat: " << std::fixed << std::setprecision(6) << (gps.lat / 1e7)
           << ", Lon: " << (gps.lon / 1e7)
           << ", Alt: " << std::setprecision(1) << (gps.alt / 1000.0f) << "m"
           << ", Sats: " << static_cast<int>(gps.satellites_visible)
           << ", Fix: " << static_cast<int>(gps.fix_type);
        logMessage(ss.str());
    }

    void handleAttitude(const mavlink_message_t& msg) {
        mavlink_attitude_t attitude;
        memcpy(&attitude, msg.payload, sizeof(attitude));
        
        std::stringstream ss;
        ss << "ATTITUDE - Roll: " << std::fixed << std::setprecision(2) 
           << (attitude.roll * 180.0f / M_PI) << "°"
           << ", Pitch: " << (attitude.pitch * 180.0f / M_PI) << "°"
           << ", Yaw: " << (attitude.yaw * 180.0f / M_PI) << "°";
        logMessage(ss.str());
    }

    void handleParamValue(const mavlink_message_t& msg) {
        mavlink_param_value_t param;
        memcpy(&param, msg.payload, sizeof(param));
        
        std::string param_id(param.param_id, 16);
        param_id = param_id.c_str(); // Remove trailing nulls
        
        std::stringstream ss;
        ss << "PARAM - ID: " << param_id
           << ", Value: " << std::fixed << std::setprecision(4) << param.param_value
           << ", Index: " << param.param_index
           << "/" << param.param_count;
        logMessage(ss.str());
    }

    void handleBatteryStatus(const mavlink_message_t& msg) {
        mavlink_battery_status_t battery;
        memcpy(&battery, msg.payload, sizeof(battery));
        
        std::stringstream ss;
        ss << "BATTERY - ID: " << static_cast<int>(battery.id)
           << ", Remaining: " << static_cast<int>(battery.battery_remaining) << "%"
           << ", Temp: " << (battery.temperature / 100.0f) << "°C";
        
        if (battery.voltages[0] != UINT16_MAX) {
            ss << ", Voltages: ";
            for (int i = 0; i < 10 && battery.voltages[i] != UINT16_MAX; i++) {
                ss << std::fixed << std::setprecision(2) << (battery.voltages[i] / 1000.0f) << "V ";
            }
        }
        logMessage(ss.str());
    }

    void handleUnknownMessage(const mavlink_message_t& msg) {
        std::stringstream ss;
        ss << "UNKNOWN MSG " << static_cast<int>(msg.msgid) << " - Raw: ";
        for (int i = 0; i < msg.len && i < 16; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(msg.payload[i]) << " ";
        }
        if (msg.len > 16) {
            ss << "...";
        }
        logMessage(ss.str());
    }
};

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    bool enable_logging = true;
    bool enable_raw_logging = false;
    
    // Simple argument parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baudrate = std::stoi(argv[++i]);
        } else if (arg == "--no-log") {
            enable_logging = false;
        } else if (arg == "--raw-log") {
            enable_raw_logging = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--port DEVICE] [--baud RATE] [--no-log] [--raw-log] [--help]" << std::endl;
            return 0;
        }
    }
    
    MavlinkUARTReader reader(enable_logging, enable_raw_logging);
    
    std::cout << "MAVLink UART Reader with Logging" << std::endl;
    std::cout << "Port: " << port << ", Baud: " << baudrate << std::endl;
    std::cout << "Logging: " << (enable_logging ? "enabled" : "disabled") << std::endl;
    std::cout << "Raw logging: " << (enable_raw_logging ? "enabled" : "disabled") << std::endl;
    std::cout << "Press Ctrl+C to stop..." << std::endl;
    
    if (!reader.initializeUART(port, baudrate)) {
        std::cerr << "Failed to initialize UART port" << std::endl;
        return -1;
    }
    
    reader.start();
    
    // Keep the program running
    try {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    return 0;
}