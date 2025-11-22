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
#include <random>
#include <functional> // Add this include at the top

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69

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
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_DEVICE_PING 0x28
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29
#define CRSF_FRAMETYPE_BIND 0x3C

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

typedef struct __mavlink_manual_control_t {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t r;
    uint16_t buttons;
    uint8_t target;
} mavlink_manual_control_t;

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
    uint8_t payload[CRSF_MAX_PACKET_LEN - 4];
    uint8_t crc;
} crsf_frame_t;

// CRSF device info structure
typedef struct __crsf_device_info_t {
    uint8_t device_address;
    uint8_t hardware_version;
    uint8_t software_version;
    uint8_t serial_number[4];
    uint8_t device_name[16];
    uint8_t device_type;
    uint8_t parameter_count;
} crsf_device_info_t;

// CRSF bind data structure
typedef struct __crsf_bind_data_t {
    uint8_t bind_type;
    uint8_t uid[6];
    uint8_t rf_mode;
    uint8_t rf_power;
    uint8_t channel_start;
    uint8_t channel_count;
} crsf_bind_data_t;

// CRC calculation for CRSF (standalone function)
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

class BidirectionalConverter {
private:
    std::atomic<bool> running;
    std::atomic<bool> binding_mode;
    int mavlink_fd;
    int crsf_fd;
    std::thread mavlink_read_thread;
    std::thread crsf_read_thread;
    
    // Current state for MAVLink to CRSF conversion
    float current_roll, current_pitch, current_yaw;
    int32_t current_lat, current_lon, current_alt;
    uint8_t current_fix_type;
    uint8_t current_satellites;
    uint16_t current_ground_speed;
    uint16_t current_heading;
    uint16_t voltage;
    uint16_t current_consumed;
    uint16_t fuel;
    
    // Current state for CRSF to MAVLink conversion
    uint16_t rc_channels[16];
    bool new_rc_data;
    
    // Binding state
    uint8_t crsf_uid[6];
    bool is_bound;
    uint32_t bind_start_time;

public:
    BidirectionalConverter() : running(false), binding_mode(false), mavlink_fd(-1), crsf_fd(-1),
                     current_roll(0), current_pitch(0), current_yaw(0),
                     current_lat(0), current_lon(0), current_alt(0),
                     current_fix_type(0), current_satellites(0),
                     current_ground_speed(0), current_heading(0),
                     voltage(0), current_consumed(0), fuel(100), 
                     new_rc_data(false), is_bound(false), bind_start_time(0) {
        memset(rc_channels, 0, sizeof(rc_channels));
        generateCRSFUID();
    }

    ~BidirectionalConverter() {
        stop();
    }

    bool initializeMAVLinkUART(const std::string& port, int baudrate = 115200) {
        mavlink_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (mavlink_fd < 0) {
            std::cerr << "Error opening MAVLink port: " << port << std::endl;
            return false;
        }
        return configureUART(mavlink_fd, baudrate, "MAVLink");
    }

    bool initializeCRSFUART(const std::string& port, int baudrate = CRSF_BAUDRATE) {
        crsf_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (crsf_fd < 0) {
            std::cerr << "Error opening CRSF port: " << port << std::endl;
            return false;
        }
        return configureUART(crsf_fd, baudrate, "CRSF");
    }

    // Start binding process
    void startBinding() {
        if (binding_mode) {
            std::cout << "Already in binding mode!" << std::endl;
            return;
        }
        
        binding_mode = true;
        bind_start_time = getCurrentTimeMs();
        std::cout << "CRSF Binding started! UID: ";
        for (int i = 0; i < 6; i++) {
            printf("%02X", crsf_uid[i]);
        }
        std::cout << std::endl;
        std::cout << "Put your transmitter in bind mode now!" << std::endl;
        
        // Send bind frame immediately
        sendCRSFBindFrame();
    }

    // Stop binding process
    void stopBinding() {
        binding_mode = false;
        std::cout << "CRSF Binding stopped" << std::endl;
    }

    // Check if currently bound
    bool isBound() const {
        return is_bound;
    }

private:
    bool configureUART(int fd, int baudrate, const std::string& name) {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting " << name << " UART attributes" << std::endl;
            close(fd);
            return false;
        }

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

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= (CS8 | CREAD | CLOCAL);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting " << name << " UART attributes" << std::endl;
            close(fd);
            return false;
        }

        std::cout << name << " UART port initialized at " << baudrate << " baud" << std::endl;
        return true;
    }

    // // Generate random CRSF UID
    // void generateCRSFUID() {
    //     std::random_device rd;
    //     std::mt19937 gen(rd());
    //     std::uniform_int_distribution<> dis(0, 255);
        
    //     for (int i = 0; i < 6; i++) {
    //         crsf_uid[i] = dis(gen);
    //     }
        
    //     // Ensure first byte is not 0x00 or 0xFF
    //     if (crsf_uid[0] == 0x00 || crsf_uid[0] == 0xFF) {
    //         crsf_uid[0] = 0xAA;
    //     }
    // }


private:
    void generateCRSFUID() {
        // Default phrase - you can change this to any string you want
        std::string phrase = "tiger";
        
        // Use std::hash to create a deterministic hash from the phrase
        std::size_t hash = std::hash<std::string>{}(phrase);
        
        // Convert the hash to 6 bytes for CRSF UID
        for (int i = 0; i < 6; i++) {
            crsf_uid[i] = (hash >> (i * 8)) & 0xFF;
        }
        
        // Ensure first byte is not 0x00 or 0xFF (reserved values)
        if (crsf_uid[0] == 0x00 || crsf_uid[0] == 0xFF) {
            crsf_uid[0] = 0xAA;
        }
        
        std::cout << "Generated CRSF UID from phrase '" << phrase << "': ";
        for (int i = 0; i < 6; i++) {
            printf("%02X", crsf_uid[i]);
        }
        std::cout << std::endl;
    }

    // ========== CRSF Binding Functions ==========

    void sendCRSFBindFrame() {
        crsf_frame_t frame;
        frame.device_address = 0xEA; // Flight controller address
        frame.frame_length = sizeof(crsf_bind_data_t) + 3;
        frame.type = CRSF_FRAMETYPE_BIND;
        
        crsf_bind_data_t bind_data;
        bind_data.bind_type = 0x01; // Receiver bind
        memcpy(bind_data.uid, crsf_uid, 6);
        bind_data.rf_mode = 0x04;   // 150Hz mode
        bind_data.rf_power = 0x1E;  // 250mW
        bind_data.channel_start = 0;
        bind_data.channel_count = 16;
        
        memcpy(frame.payload, &bind_data, sizeof(bind_data));
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, sizeof(bind_data));
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
        std::cout << "Sent CRSF bind frame" << std::endl;
    }

    void sendCRSFDeviceInfo() {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = sizeof(crsf_device_info_t) + 3;
        frame.type = CRSF_FRAMETYPE_DEVICE_INFO;
        
        crsf_device_info_t device_info;
        device_info.device_address = 0xEA;
        device_info.hardware_version = 0x01;
        device_info.software_version = 0x01;
        memcpy(device_info.serial_number, crsf_uid, 4);
        
        const char* device_name = "MAVLink2CRSF";
        memcpy(device_info.device_name, device_name, strlen(device_name));
        memset(device_info.device_name + strlen(device_name), 0, 16 - strlen(device_name));
        
        device_info.device_type = 0x0E; // Flight controller
        device_info.parameter_count = 0;
        
        memcpy(frame.payload, &device_info, sizeof(device_info));
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, sizeof(device_info));
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    void handleCRSFBindFrame(const crsf_frame_t& frame) {
        if (frame.type == CRSF_FRAMETYPE_BIND) {
            std::cout << "Received bind frame - checking UID match..." << std::endl;
            
            // Check if this is a bind response with matching UID
            if (frame.frame_length >= 10) {
                crsf_bind_data_t bind_data;
                memcpy(&bind_data, frame.payload, sizeof(bind_data));
                
                // Check if UID matches ours
                bool uid_match = true;
                for (int i = 0; i < 6; i++) {
                    if (bind_data.uid[i] != crsf_uid[i]) {
                        uid_match = false;
                        break;
                    }
                }
                
                if (uid_match) {
                    is_bound = true;
                    binding_mode = false;
                    std::cout << "CRSF Binding successful! Receiver is now bound." << std::endl;
                    std::cout << "RF Mode: " << static_cast<int>(bind_data.rf_mode);
                    std::cout << ", Power: " << static_cast<int>(bind_data.rf_power) << "mW" << std::endl;
                }
            }
        }
    }

    // ========== MAVLINK to CRSF Conversion ==========

    void sendCRSFAttitude() {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = 7 + 3;
        frame.type = CRSF_FRAMETYPE_ATTITUDE;
        
        int16_t pitch = static_cast<int16_t>(current_pitch * 10000.0f);
        int16_t roll = static_cast<int16_t>(current_roll * 10000.0f);
        int16_t yaw = static_cast<int16_t>(current_yaw * 10000.0f);
        
        frame.payload[0] = (pitch >> 8) & 0xFF;
        frame.payload[1] = pitch & 0xFF;
        frame.payload[2] = (roll >> 8) & 0xFF;
        frame.payload[3] = roll & 0xFF;
        frame.payload[4] = (yaw >> 8) & 0xFF;
        frame.payload[5] = yaw & 0xFF;
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 6);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    void sendCRSFGPS() {
        if (current_fix_type < 2) return;
        
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = 16 + 3;
        frame.type = CRSF_FRAMETYPE_GPS;
        
        int32_t lat = current_lat;
        int32_t lon = current_lon;
        int32_t alt = current_alt;
        uint16_t speed = current_ground_speed;
        uint16_t heading = current_heading;
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
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 16);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    void sendCRSFBattery() {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        frame.frame_length = 9 + 3;
        frame.type = CRSF_FRAMETYPE_BATTERY_SENSOR;
        
        frame.payload[0] = (voltage >> 8) & 0xFF;
        frame.payload[1] = voltage & 0xFF;
        
        frame.payload[2] = (current_consumed >> 8) & 0xFF;
        frame.payload[3] = current_consumed & 0xFF;
        
        frame.payload[4] = 0;
        frame.payload[5] = 0;
        
        frame.payload[6] = fuel;
        frame.payload[7] = 0;
        frame.payload[8] = 0;
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, 9);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

    void sendCRSFFlightMode(const char* mode) {
        crsf_frame_t frame;
        frame.device_address = 0xEA;
        
        size_t mode_len = strlen(mode);
        if (mode_len > 16) mode_len = 16;
        
        frame.frame_length = mode_len + 3;
        frame.type = CRSF_FRAMETYPE_FLIGHT_MODE;
        
        memcpy(frame.payload, mode, mode_len);
        
        uint8_t crc_data[frame.frame_length - 1];
        crc_data[0] = frame.device_address;
        crc_data[1] = frame.frame_length;
        crc_data[2] = frame.type;
        memcpy(&crc_data[3], frame.payload, mode_len);
        
        frame.crc = crsf_crc8(crc_data, frame.frame_length - 1);
        
        sendCRSFFrame(&frame);
    }

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

    // ========== CRSF to MAVLINK Conversion ==========

    void sendMAVLinkRCChannels() {
        if (!new_rc_data) return;
        
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.magic = 0xFE;
        msg.len = 42;
        msg.seq = 0;
        msg.sysid = 1;
        msg.compid = 1;
        msg.msgid = MAVLINK_MSG_ID_RC_CHANNELS;
        
        mavlink_rc_channels_t rc_channels;
        rc_channels.time_boot_ms = getCurrentTimeMs();
        for (int i = 0; i < 16; i++) {
            ((uint16_t*)&rc_channels)[i] = this->rc_channels[i];
        }
        rc_channels.chancount = 16;
        rc_channels.rssi = is_bound ? 255 : 0;
        
        memcpy(msg.payload, &rc_channels, sizeof(rc_channels));
        
        sendMAVLinkMessage(&msg);
        new_rc_data = false;
        
        // Debug output
        std::cout << "RC Channels to MAVLink: ";
        for (int i = 0; i < 4; i++) {
            std::cout << "Ch" << i+1 << ":" << this->rc_channels[i] << " ";
        }
        std::cout << std::endl;
    }

    void sendMAVLinkManualControl() {
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.magic = 0xFE;
        msg.len = 11;
        msg.seq = 0;
        msg.sysid = 1;
        msg.compid = 1;
        msg.msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
        
        mavlink_manual_control_t manual;
        manual.target = 1;
        manual.x = mapRCtoManual(rc_channels[0]);
        manual.y = mapRCtoManual(rc_channels[1]);
        manual.z = mapRCtoManual(rc_channels[2]);
        manual.r = mapRCtoManual(rc_channels[3]);
        manual.buttons = 0;
        
        memcpy(msg.payload, &manual, sizeof(manual));
        
        sendMAVLinkMessage(&msg);
    }

    int16_t mapRCtoManual(uint16_t rc_value) {
        if (rc_value < 1000) rc_value = 1000;
        if (rc_value > 2000) rc_value = 2000;
        return static_cast<int16_t>((rc_value - 1500) * 2);
    }

    void sendMAVLinkMessage(mavlink_message_t* msg) {
        if (mavlink_fd < 0) return;
        
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        int pos = 0;
        
        buffer[pos++] = msg->magic;
        buffer[pos++] = msg->len;
        buffer[pos++] = msg->seq;
        buffer[pos++] = msg->sysid;
        buffer[pos++] = msg->compid;
        buffer[pos++] = msg->msgid;
        
        memcpy(&buffer[pos], msg->payload, msg->len);
        pos += msg->len;
        
        uint8_t checksum = 0;
        for (int i = 1; i < pos; i++) {
            checksum ^= buffer[i];
        }
        buffer[pos++] = checksum;
        
        write(mavlink_fd, buffer, pos);
    }

    // ========== Parsers ==========

    class CRSFParser {
    private:
        enum ParseState { WAITING_SYNC, READING_FRAME };
        ParseState state;
        uint8_t buffer[CRSF_MAX_PACKET_LEN];
        uint8_t frame_index;
        uint8_t expected_length;
        
    public:
        CRSFParser() : state(WAITING_SYNC), frame_index(0), expected_length(0) {}
        
        bool parse_char(uint8_t c, crsf_frame_t& frame) {
            switch (state) {
                case WAITING_SYNC:
                    if (c == CRSF_SYNC_BYTE) {
                        state = READING_FRAME;
                        frame_index = 0;
                        buffer[frame_index++] = c;
                    }
                    break;
                    
                case READING_FRAME:
                    buffer[frame_index++] = c;
                    
                    if (frame_index >= 3) {
                        expected_length = buffer[2];
                        
                        if (frame_index >= expected_length + 2) {
                            memcpy(&frame, &buffer[1], sizeof(frame));
                            state = WAITING_SYNC;
                            
                            // Use the standalone crsf_crc8 function
                            uint8_t crc = crsf_crc8(&buffer[1], expected_length + 1);
                            if (crc == frame.crc) {
                                return true;
                            }
                        }
                    }
                    break;
            }
            return false;
        }
    };

    class MavlinkParser {
    private:
        enum ParseState { WAITING_START, GOT_START, GOT_LENGTH, GOT_SEQ, GOT_SYSID, GOT_COMPID, GOT_MSGID, READING_PAYLOAD, GOT_CRC1, GOT_CRC2 };
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
                    if (c == 0xFE) state = GOT_START; 
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
                    state = (expected_length > 0) ? READING_PAYLOAD : GOT_CRC1; 
                    break;
                case READING_PAYLOAD: 
                    if (payload_index < expected_length) 
                        current_msg.payload[payload_index++] = c;
                    if (payload_index >= expected_length) 
                        state = GOT_CRC1;
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

    // ========== Utility Functions ==========

    uint32_t getCurrentTimeMs() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }

    uint16_t decodeCRSFChannel(const uint8_t* data, int channel) {
        int bit_index = channel * 11;
        int byte_index = bit_index / 8;
        int bit_offset = bit_index % 8;
        
        uint32_t bits = (data[byte_index] << 16) | (data[byte_index + 1] << 8) | data[byte_index + 2];
        bits >>= (8 - bit_offset);
        bits &= 0x7FF;
        
        return static_cast<uint16_t>(((bits * 1000) >> 11) + 1000);
    }

public:
    void start() {
        running = true;
        mavlink_read_thread = std::thread(&BidirectionalConverter::mavlinkReadLoop, this);
        crsf_read_thread = std::thread(&BidirectionalConverter::crsfReadLoop, this);
    }

    void stop() {
        running = false;
        if (mavlink_read_thread.joinable()) mavlink_read_thread.join();
        if (crsf_read_thread.joinable()) crsf_read_thread.join();
        if (mavlink_fd >= 0) { close(mavlink_fd); mavlink_fd = -1; }
        if (crsf_fd >= 0) { close(crsf_fd); crsf_fd = -1; }
    }

private:
    void mavlinkReadLoop() {
        MavlinkParser parser;
        mavlink_message_t msg;
        uint8_t buffer[256];
        
        std::cout << "MAVLink read loop started" << std::endl;

        auto last_send = std::chrono::steady_clock::now();
        
        while (running) {
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
            
            // Send CRSF telemetry periodically (10Hz)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
            
            if (elapsed.count() >= 100) {
                if (is_bound) {
                    sendCRSFAttitude();
                    sendCRSFGPS();
                    sendCRSFBattery();
                    sendCRSFFlightMode("MANUAL");
                }
                last_send = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void crsfReadLoop() {
        CRSFParser parser;
        crsf_frame_t frame;
        uint8_t buffer[256];
        
        std::cout << "CRSF read loop started" << std::endl;

        auto last_send = std::chrono::steady_clock::now();
        auto last_bind_send = std::chrono::steady_clock::now();
        auto last_device_info = std::chrono::steady_clock::now();
        
        while (running) {
            if (crsf_fd >= 0) {
                ssize_t n = read(crsf_fd, buffer, sizeof(buffer));
                if (n > 0) {
                    for (ssize_t i = 0; i < n; i++) {
                        if (parser.parse_char(buffer[i], frame)) {
                            handleCRSFFrame(frame);
                        }
                    }
                }
            }
            
            auto now = std::chrono::steady_clock::now();
            
            // Handle binding mode
            if (binding_mode) {
                auto bind_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_bind_send);
                if (bind_elapsed.count() >= 1000) { // Send bind frame every second
                    sendCRSFBindFrame();
                    last_bind_send = now;
                }
                
                // Timeout binding after 60 seconds
                if (getCurrentTimeMs() - bind_start_time > 60000) {
                    std::cout << "Binding timeout - no transmitter found" << std::endl;
                    binding_mode = false;
                }
            }
            
            // Send device info periodically
            auto device_info_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_device_info);
            if (device_info_elapsed.count() >= 5000) { // Every 5 seconds
                sendCRSFDeviceInfo();
                last_device_info = now;
            }
            
            // Send MAVLink RC data periodically (50Hz)
            auto rc_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
            if (rc_elapsed.count() >= 20) {
                sendMAVLinkRCChannels();
                sendMAVLinkManualControl();
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
            case MAVLINK_MSG_ID_HEARTBEAT:
                handleHeartbeat(msg);
                break;
            default:
                //std::cout << "MAVLink Msg ID: " << static_cast<int>(msg.msgid) << std::endl;
                break;
        }
    }

    void handleCRSFFrame(const crsf_frame_t& frame) {
        switch (frame.type) {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                handleCRSFRCChannels(frame);
                break;
            case CRSF_FRAMETYPE_BIND:
                handleCRSFBindFrame(frame);
                break;
            case CRSF_FRAMETYPE_DEVICE_PING:
                std::cout << "Received device ping" << std::endl;
                sendCRSFDeviceInfo();
                break;
            default:
                //std::cout << "CRSF Frame Type: 0x" << std::hex << static_cast<int>(frame.type) << std::dec << std::endl;
                break;
        }
    }

    void handleAttitude(const mavlink_message_t& msg) {
        mavlink_attitude_t attitude;
        memcpy(&attitude, msg.payload, sizeof(attitude));
        
        current_roll = attitude.roll;
        current_pitch = attitude.pitch;
        current_yaw = attitude.yaw;
        
        std::cout << "Attitude - Roll: " << current_roll << " Pitch: " << current_pitch << " Yaw: " << current_yaw << std::endl;
    }

    void handleGPSRawInt(const mavlink_message_t& msg) {
        mavlink_gps_raw_int_t gps;
        memcpy(&gps, msg.payload, sizeof(gps));
        
        current_lat = gps.lat;
        current_lon = gps.lon;
        current_alt = gps.alt;
        current_fix_type = gps.fix_type;
        current_satellites = gps.satellites_visible;
        current_ground_speed = gps.vel;
        current_heading = gps.cog;
        
        std::cout << "GPS - Lat: " << (current_lat/1e7) << " Lon: " << (current_lon/1e7) 
                  << " Alt: " << (current_alt/1000.0f) << "m" << std::endl;
    }

    void handleHeartbeat(const mavlink_message_t& msg) {
        mavlink_heartbeat_t heartbeat;
        memcpy(&heartbeat, msg.payload, sizeof(heartbeat));
        
        voltage = 1680;
        current_consumed = 1500;
        fuel = 85;
        
        std::cout << "Heartbeat from Pixhawk" << std::endl;
    }

    void handleCRSFRCChannels(const crsf_frame_t& frame) {
        for (int i = 0; i < 16; i++) {
            rc_channels[i] = decodeCRSFChannel(frame.payload, i);
        }
        new_rc_data = true;
        
        if (!is_bound) {
            is_bound = true;
            std::cout << "CRSF link established! Receiving RC data." << std::endl;
        }
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
    int crsf_baud = 115200; // Using 115200 instead of 420000 for compatibility
    bool auto_bind = false;
    
    // Parse command line arguments
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
        } else if (arg == "--bind") {
            auto_bind = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--mavlink-port DEVICE] [--crsf-port DEVICE] [--mavlink-baud RATE] [--crsf-baud RATE] [--bind]" << std::endl;
            std::cout << "Default: MAVLink=" << mavlink_port << ":" << mavlink_baud 
                      << ", CRSF=" << crsf_port << ":" << crsf_baud << std::endl;
            std::cout << "Interactive commands while running:" << std::endl;
            std::cout << "  'b' - Start binding" << std::endl;
            std::cout << "  's' - Stop binding" << std::endl;
            std::cout << "  'q' - Quit" << std::endl;
            return 0;
        }
    }
    
    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    
    BidirectionalConverter converter;
    
    std::cout << "==========================================" << std::endl;
    std::cout << "  CRSF-MAVLink Bridge with Binding" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "MAVLink: " << mavlink_port << " at " << mavlink_baud << " baud" << std::endl;
    std::cout << "CRSF:    " << crsf_port << " at " << crsf_baud << " baud" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Interactive commands:" << std::endl;
    std::cout << "  'b' - Start CRSF binding" << std::endl;
    std::cout << "  's' - Stop binding" << std::endl;
    std::cout << "  'q' - Quit application" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "==========================================" << std::endl;
    
    // Initialize UART ports
    if (!converter.initializeMAVLinkUART(mavlink_port, mavlink_baud)) {
        std::cerr << "Failed to initialize MAVLink UART" << std::endl;
        return -1;
    }
    
    if (!converter.initializeCRSFUART(crsf_port, crsf_baud)) {
        std::cerr << "Failed to initialize CRSF UART" << std::endl;
        return -1;
    }
    
    // Auto-start binding if requested
    if (auto_bind) {
        std::cout << "Auto-binding enabled, starting binding process..." << std::endl;
        converter.startBinding();
    }
    
    // Start the converter threads
    converter.start();
    std::cout << "Bridge started successfully!" << std::endl;
    
    // Main loop with interactive input handling
    while (!shutdown_requested) {
        struct timeval tv = {0, 100000}; // 100ms timeout
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        
        int ready = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        
        if (ready > 0) {
            char c;
            read(STDIN_FILENO, &c, 1);
            switch (c) {
                case 'b':
                case 'B':
                    converter.startBinding();
                    break;
                case 's':
                case 'S':
                    converter.stopBinding();
                    break;
                case 'q':
                case 'Q':
                    shutdown_requested = true;
                    break;
                default:
                    std::cout << "Unknown command '" << c << "'. Use 'b'=bind, 's'=stop, 'q'=quit" << std::endl;
                    break;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean shutdown
    converter.stop();
    std::cout << "\nBridge stopped gracefully." << std::endl;
    
    return 0;
}