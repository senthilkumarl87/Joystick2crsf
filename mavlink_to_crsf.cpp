#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <map>
#include <csignal>
#include <sys/select.h>

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_GPS_RAW_INT 24

#define MAVLINK_MAX_PACKET_LEN 263

// CRSF protocol constants
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_BAUDRATE 420000

// CRSF frame types
#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21

// MAVLink message structures
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

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
    uint8_t chancount;
    uint8_t rssi;
} mavlink_rc_channels_t;

typedef struct __mavlink_attitude_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} mavlink_attitude_t;

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

// CRSF frame structure
typedef struct __crsf_frame {
    uint8_t device_address;
    uint8_t frame_length;
    uint8_t type;
    uint8_t payload[CRSF_MAX_PACKET_LEN - 4]; // -4 for addr, len, type, crc
    uint8_t crc;
} crsf_frame_t;

class CRSFConverter {
private:
    std::atomic<bool> running;
    int mavlink_fd;
    int crsf_fd;
    std::thread read_thread;
    
    // Current state
    float current_roll, current_pitch, current_yaw;
    int32_t current_lat, current_lon, current_alt;
    uint8_t current_fix_type;
    uint8_t current_satellites;
    uint16_t current_ground_speed;
    uint16_t current_heading;
    uint16_t voltage; // in centivolts
    uint16_t current; // in centiamps
    uint16_t fuel; // in percent

public:
    CRSFConverter() : running(false), mavlink_fd(-1), crsf_fd(-1),
                     current_roll(0), current_pitch(0), current_yaw(0),
                     current_lat(0), current_lon(0), current_alt(0),
                     current_fix_type(0), current_satellites(0),
                     current_ground_speed(0), current_heading(0),
                     voltage(0), current(0), fuel(100) {}

    ~CRSFConverter() {
        stop();
    }

    // Initialize MAVLink UART (from Pixhawk)
    bool initializeMAVLinkUART(const std::string& port, int baudrate = 115200) {
        mavlink_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (mavlink_fd < 0) {
            std::cerr << "Error opening MAVLink port: " << port << std::endl;
            return false;
        }

        return configureUART(mavlink_fd, baudrate, "MAVLink");
    }

    // Initialize CRSF UART (output)
    bool initializeCRSFUART(const std::string& port, int baudrate = CRSF_BAUDRATE) {
        crsf_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (crsf_fd < 0) {
            std::cerr << "Error opening CRSF port: " << port << std::endl;
            return false;
        }

        return configureUART(crsf_fd, baudrate, "CRSF");
    }

private:
    bool configureUART(int fd, int baudrate, const std::string& name) {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting " << name << " UART attributes" << std::endl;
            close(fd);
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
            case 420000: 
                // CRSF typically uses 420000 baud
                #ifdef B400000
                speed = B400000;
                #else
                std::cerr << "420000 baud not supported, using 115200" << std::endl;
                speed = B115200;
                #endif
                break;
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
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 0.1 second timeout

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting " << name << " UART attributes" << std::endl;
            close(fd);
            return false;
        }

        std::cout << name << " UART port initialized at " << baudrate << " baud" << std::endl;
        return true;
    }

    // Calculate CRC for CRSF frame
    uint8_t crsf_crc8(const uint8_t* data, size_t len) {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    // Convert MAVLink attitude to CRSF attitude frame
    void sendCRSFAttitude() {
        crsf_frame_t frame;
        frame.device_address = 0xEA; // Flight controller address
        frame.frame_length = 7 + 3; // type + payload + crc
        frame.type = CRSF_FRAMETYPE_ATTITUDE;
        
        // Convert radians to degrees * 10000
        int16_t pitch = static_cast<int16_t>(current_pitch * 10000.0f);
        int16_t roll = static_cast<int16_t>(current_roll * 10000.0f);
        int16_t yaw = static_cast<int16_t>(current_yaw * 10000.0f);
        
        frame.payload[0] = (pitch >> 8) & 0xFF;
        frame.payload[1] = pitch & 0xFF;
        frame.payload[2] = (roll >> 8) & 0xFF;
        frame.payload[3] = roll & 0xFF;
        frame.payload[4] = (yaw >> 8) & 0xFF;
        frame.payload[5] = yaw & 0xFF;
        
        // Calculate CRC for the frame (excluding sync byte)
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 6);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    // Convert MAVLink GPS to CRSF GPS frame
    void sendCRSFGPS() {
        if (current_fix_type < 2) return; // Only send if we have 2D or 3D fix
        
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = 16 + 3; // type + payload + crc
        frame.type = CRSF_FRAMETYPE_GPS;
        
        // Latitude and longitude in degrees * 1e7
        int32_t lat = current_lat;
        int32_t lon = current_lon;
        int32_t alt = current_alt; // in mm
        uint16_t speed = current_ground_speed; // in cm/s
        uint16_t heading = current_heading; // in degrees * 100
        uint8_t satellites = current_satellites;
        
        frame.payload[0] = (lat >> 24) & 0xFF;
        frame.payload[1] = (lat >> 16) & 0xFF;
        frame.payload[2] = (lat >> 8) & 0xFF;
        frame.payload[3] = lat & 0xFF;
        
        frame.payload[4] = (lon >> 24) & 0xFF;
        frame.payload[5] = (lon >> 16) & 0xFF;
        frame.payload[6] = (lon >> 8) & 0xFF;
        frame.payload[7] = lon & 0xFF;
        
        frame.payload[8] = (speed >> 8) & 0xFF;
        frame.payload[9] = speed & 0xFF;
        
        frame.payload[10] = (heading >> 8) & 0xFF;
        frame.payload[11] = heading & 0xFF;
        
        frame.payload[12] = (alt >> 16) & 0xFF;
        frame.payload[13] = (alt >> 8) & 0xFF;
        frame.payload[14] = alt & 0xFF;
        
        frame.payload[15] = satellites;
        
        // Calculate CRC
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 16);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    // Send battery information as CRSF frame
    void sendCRSFBattery() {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = 9 + 3; // type + payload + crc
        frame.type = CRSF_FRAMETYPE_BATTERY_SENSOR;
        
        // Voltage in centivolts (0.01V units)
        frame.payload[0] = (voltage >> 8) & 0xFF;
        frame.payload[1] = voltage & 0xFF;
        
        // Current in centiamps (0.01A units)
        frame.payload[2] = (current >> 8) & 0xFF;
        frame.payload[3] = current & 0xFF;
        
        // Fuel (capacity) in mAh (not used here, set to 0)
        frame.payload[4] = 0;
        frame.payload[5] = 0;
        
        // Battery remaining in percent
        frame.payload[6] = fuel;
        
        // Unused bytes
        frame.payload[7] = 0;
        frame.payload[8] = 0;
        
        // Calculate CRC
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 9);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    // Send flight mode as CRSF frame
    void sendCRSFFlightMode(const char* mode) {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        
        size_t mode_len = strlen(mode);
        if (mode_len > 16) mode_len = 16; // CRSF limits flight mode to 16 chars
        
        frame.frame_length = mode_len + 3; // type + payload + crc
        frame.type = CRSF_FRAMETYPE_FLIGHT_MODE;
        
        memcpy(frame.payload, mode, mode_len);
        
        // Calculate CRC
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, mode_len);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    // Send complete CRSF frame
    void sendCRSFFrame(crsf_frame_t* frame) {
        if (crsf_fd < 0) return;
        
        uint8_t buffer[CRSF_MAX_PACKET_LEN];
        int pos = 0;
        
        buffer[pos++] = CRSF_SYNC_BYTE;
        buffer[pos++] = frame->device_address;
        buffer[pos++] = frame->frame_length;
        buffer[pos++] = frame->type;
        
        memcpy(&buffer[pos], frame->payload, frame->frame_length - 3);
        pos += frame->frame_length - 3;
        
        buffer[pos++] = frame->crc;
        
        write(crsf_fd, buffer, pos);
    }

    // MAVLink parser
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
                    if (c == 0xFE) {
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
                    state = GOT_CRC2;
                    break;
                    
                case GOT_CRC2:
                    msg = current_msg;
                    state = WAITING_START;
                    return true;
            }
            
            return false;
        }
    };

public:
    void start() {
        running = true;
        read_thread = std::thread(&CRSFConverter::readLoop, this);
    }

    void stop() {
        running = false;
        if (read_thread.joinable()) {
            read_thread.join();
        }
        if (mavlink_fd >= 0) {
            close(mavlink_fd);
            mavlink_fd = -1;
        }
        if (crsf_fd >= 0) {
            close(crsf_fd);
            crsf_fd = -1;
        }
    }

private:
    void readLoop() {
        MavlinkParser parser;
        mavlink_message_t msg;
        uint8_t buffer[256];
        
        std::cout << "Starting MAVLink to CRSF conversion..." << std::endl;

        auto last_send = std::chrono::steady_clock::now();
        
        while (running) {
            // Read from MAVLink
            if (mavlink_fd >= 0) {
                ssize_t n = read(mavlink_fd, buffer, sizeof(buffer));
                
                if (n > 0) {
                    for (ssize_t i = 0; i < n; i++) {
                        if (parser.parse_char(buffer[i], msg)) {
                            handleMavlinkMessage(msg);
                        }
                    }
                }
            }
            
            // Send CRSF frames periodically (approx 10Hz)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
            
            if (elapsed.count() >= 100) { // 10Hz
                sendCRSFAttitude();
                sendCRSFGPS();
                sendCRSFBattery();
                
                // Determine flight mode from custom_mode
                const char* flight_mode = "MANUAL";
                if (current_roll != 0 || current_pitch != 0) { // Simple heuristic
                    flight_mode = "ACRO";
                }
                sendCRSFFlightMode(flight_mode);
                
                last_send = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void handleMavlinkMessage(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_ATTITUDE:
                handleAttitude(msg);
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                handleGPSRawInt(msg);
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS:
                handleRCChannels(msg);
                break;
            case MAVLINK_MSG_ID_HEARTBEAT:
                handleHeartbeat(msg);
                break;
        }
    }

    void handleAttitude(const mavlink_message_t& msg) {
        mavlink_attitude_t attitude;
        memcpy(&attitude, msg.payload, sizeof(attitude));
        
        current_roll = attitude.roll;
        current_pitch = attitude.pitch;
        current_yaw = attitude.yaw;
        
        std::cout << "Attitude - R:" << current_roll << " P:" << current_pitch << " Y:" << current_yaw << std::endl;
    }

    void handleGPSRawInt(const mavlink_message_t& msg) {
        mavlink_gps_raw_int_t gps;
        memcpy(&gps, msg.payload, sizeof(gps));
        
        current_lat = gps.lat;
        current_lon = gps.lon;
        current_alt = gps.alt;
        current_fix_type = gps.fix_type;
        current_satellites = gps.satellites_visible;
        current_ground_speed = gps.vel; // cm/s
        current_heading = gps.cog; // degrees * 100
        
        std::cout << "GPS - Lat:" << (current_lat/1e7) << " Lon:" << (current_lon/1e7) 
                  << " Alt:" << (current_alt/1000.0f) << "m" << std::endl;
    }

    void handleRCChannels(const mavlink_message_t& msg) {
        mavlink_rc_channels_t rc_channels;
        memcpy(&rc_channels, msg.payload, sizeof(rc_channels));
        
        // You could convert RC channels to CRSF if needed
        // CRSF typically doesn't send RC inputs back, but you could if required
    }

    void handleHeartbeat(const mavlink_message_t& msg) {
        mavlink_heartbeat_t heartbeat;
        memcpy(&heartbeat, msg.payload, sizeof(heartbeat));
        
        // Update battery simulation (in real implementation, use SYS_STATUS message)
        voltage = 1680; // 16.8V in centivolts
        current = 1500; // 15A in centiamps
        fuel = 85; // 85% remaining
        
        std::cout << "Heartbeat - Mode:" << heartbeat.custom_mode << std::endl;
    }
};

// Signal handler
static std::atomic<bool> shutdown_requested(false);
static void signal_handler(int signal) {
    shutdown_requested = true;
}

int main(int argc, char* argv[]) {
    std::string mavlink_port = "/dev/ttyACM0";
    std::string crsf_port = "/dev/ttyUSB0";
    int mavlink_baud = 57600;
    int crsf_baud = 420000;
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--mavlink-port" && i + 1 < argc) {
            mavlink_port = argv[++i];
        } else if (arg == "--crsf-port" && i + 1 < argc) {
            crsf_port = argv[++i];
        } else if (arg == "--mavlink-baud" && i + 1 < argc) {
            mavlink_baud = std::stoi(argv[++i]);
        } else if (arg == "--crsf-baud" && i + 1 < argc) {
            crsf_baud = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--mavlink-port DEVICE] [--crsf-port DEVICE] [--mavlink-baud RATE] [--crsf-baud RATE]" << std::endl;
            return 0;
        }
    }
    
    std::signal(SIGINT, signal_handler);
    
    CRSFConverter converter;
    
    std::cout << "MAVLink to CRSF Converter" << std::endl;
    std::cout << "MAVLink: " << mavlink_port << " at " << mavlink_baud << " baud" << std::endl;
    std::cout << "CRSF: " << crsf_port << " at " << crsf_baud << " baud" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    if (!converter.initializeMAVLinkUART(mavlink_port, mavlink_baud)) {
        std::cerr << "Failed to initialize MAVLink UART" << std::endl;
        return -1;
    }
    
    if (!converter.initializeCRSFUART(crsf_port, crsf_baud)) {
        std::cerr << "Failed to initialize CRSF UART" << std::endl;
        return -1;
    }
    
    converter.start();
    
    while (!shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    converter.stop();
    std::cout << "\nConverter stopped" << std::endl;
    
    return 0;
}