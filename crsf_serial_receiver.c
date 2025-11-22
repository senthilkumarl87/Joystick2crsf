#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <termios.h>
#include <sys/ioctl.h>

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define MAX_FRAME_SIZE 64
#define UART_BUFFER_SIZE 256

volatile sig_atomic_t running = 1;

void signal_handler(int signal) {
    running = 0;
}

struct CRSFChannelData {
    uint16_t channels[16];
    int num_channels;
    uint32_t frame_count;
    uint64_t timestamp;
};

struct UARTConfig {
    int fd;
    int enabled;
    char device[64];
    int baud_rate;
    int bytes_received;
};

struct CRSFReceiver {
    // UART configuration
    struct UARTConfig uart;
    
    // Statistics
    uint32_t total_frames;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t sync_errors;
    uint32_t uart_errors;
    uint64_t start_time;
    
    // Current channel data
    struct CRSFChannelData current_channels;
    
    // UART receive buffer
    uint8_t uart_buffer[UART_BUFFER_SIZE];
    size_t uart_buffer_pos;
};

// Forward declarations
void print_channels(const struct CRSFChannelData* channels);
void print_statistics(const struct CRSFReceiver *receiver);

// CRC8 function matching CRSF standard
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

int crsf_validate_frame(const uint8_t* frame, size_t frame_len) {
    if (frame_len < 4) return 0;
    
    // Check sync byte
    if (frame[0] != CRSF_SYNC_BYTE) return 0;
    
    // Check frame size
    uint8_t frame_size_byte = frame[1];
    size_t expected_len = frame_size_byte + 2;
    
    if (frame_len != expected_len) return 0;
    
    // Check CRC
    uint8_t calculated_crc = crc8_data(frame + 2, frame_len - 3);
    uint8_t frame_crc = frame[frame_len - 1];
    
    return calculated_crc == frame_crc;
}

// Convert CRSF value (0-1984) to RC value (1000-2000)
uint16_t crsf_to_rc(uint16_t crsf_value) {
    return (uint16_t)((crsf_value / 1.984) + 1000);
}

// Unpack 11-bit channels from 22-byte payload
int unpack_11bit_channels(uint16_t* channels, const uint8_t* payload, size_t payload_len) {
    if (payload_len < 22) return 0;
    
    for (int i = 0; i < 16; i++) {
        int bit_pos = i * 11;
        int byte_index = bit_pos / 8;
        int bit_index = bit_pos % 8;
        
        if (byte_index + 2 >= payload_len) {
            channels[i] = 992; // Default center
            continue;
        }
        
        if (bit_index <= 5) {
            // Bits fit within 2 bytes
            uint16_t low_byte = payload[byte_index];
            uint16_t high_byte = payload[byte_index + 1];
            
            channels[i] = ((low_byte >> bit_index) | (high_byte << (8 - bit_index))) & 0x7FF;
        } else {
            // Bits span 3 bytes
            uint16_t low_byte = payload[byte_index];
            uint16_t mid_byte = payload[byte_index + 1];
            uint16_t high_byte = payload[byte_index + 2];
            
            channels[i] = ((low_byte >> bit_index) |
                          (mid_byte << (8 - bit_index)) |
                          (high_byte << (16 - bit_index))) & 0x7FF;
        }
    }
    
    return 16;
}

// Get current timestamp in microseconds
uint64_t get_timestamp_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

// Get formatted time string
void get_time_string(char* buffer, size_t buffer_size) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buffer, buffer_size, "%H:%M:%S", tm_info);
}

// Convert baud rate constant to speed_t
speed_t get_baud_rate_constant(int baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:
            printf("⚠️  Unsupported baud rate %d, using 115200\n", baud_rate);
            return B115200;
    }
}

// UART functions
int uart_init(struct UARTConfig *uart, const char* device, int baud_rate) {
    uart->fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart->fd < 0) {
        perror("Error opening serial port");
        return -1;
    }
    
    struct termios options;
    
    // Get current serial port settings
    if (tcgetattr(uart->fd, &options) < 0) {
        perror("Error getting serial port settings");
        close(uart->fd);
        return -1;
    }
    
    // Set baud rate
    speed_t speed = get_baud_rate_constant(baud_rate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;    // No parity
    options.c_cflag &= ~CSTOPB;    // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8 data bits
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    options.c_oflag &= ~OPOST;
    
    // No blocking, return immediately with what's available
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    
    // Apply settings
    if (tcsetattr(uart->fd, TCSANOW, &options) < 0) {
        perror("Error setting serial port settings");
        close(uart->fd);
        return -1;
    }
    
    // Flush any existing data
    tcflush(uart->fd, TCIOFLUSH);
    
    strncpy(uart->device, device, sizeof(uart->device) - 1);
    uart->baud_rate = baud_rate;
    uart->enabled = 1;
    uart->bytes_received = 0;
    
    printf("UART initialized: %s at %d baud\n", device, baud_rate);
    return 0;
}

void uart_cleanup(struct UARTConfig *uart) {
    if (uart->fd >= 0) {
        close(uart->fd);
        uart->fd = -1;
    }
}

int uart_receive_bytes(struct UARTConfig *uart, uint8_t* buffer, size_t buffer_size) {
    if (!uart->enabled || uart->fd < 0) {
        return -1;
    }
    
    ssize_t received = read(uart->fd, buffer, buffer_size);
    
    if (received > 0) {
        uart->bytes_received += received;
        return received;
    }
    
    return -1;
}

// Receiver initialization
void receiver_init(struct CRSFReceiver *receiver, const char* uart_device, int baud_rate) {
    memset(receiver, 0, sizeof(*receiver));
    
    // Initialize UART
    if (uart_init(&receiver->uart, uart_device, baud_rate) == 0) {
        printf("📡 UART receiver enabled on %s at %d baud\n", receiver->uart.device, receiver->uart.baud_rate);
    } else {
        fprintf(stderr, "❌ UART initialization failed\n");
        receiver->uart.enabled = 0;
    }
    
    // Initialize channel data
    for (int i = 0; i < 16; i++) {
        receiver->current_channels.channels[i] = 992; // Center position
    }
    receiver->current_channels.num_channels = 16;
    receiver->current_channels.frame_count = 0;
    
    receiver->uart_buffer_pos = 0;
    receiver->start_time = get_timestamp_us();
    
    printf("CRSF UART Receiver Initialized\n");
    printf("Frame types supported: RC_CHANNELS_PACKED (0x16)\n");
    printf("==========================================\n");
}

void receiver_cleanup(struct CRSFReceiver *receiver) {
    uart_cleanup(&receiver->uart);
}

// Process received CRSF frame
int process_crsf_frame(struct CRSFReceiver *receiver, const uint8_t* frame, size_t frame_len) {
    receiver->total_frames++;
    
    // Validate frame
    if (!crsf_validate_frame(frame, frame_len)) {
        receiver->crc_errors++;
        return -1;
    }
    
    // Check frame type
    uint8_t frame_type = frame[2];
    
    if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // Process RC channels packet
        if (frame_len < 26) {
            printf("   ❌ RC channels packet too short: %zu bytes\n", frame_len);
            return -1;
        }
        
        // Extract payload (22 bytes for 16 channels)
        const uint8_t* payload = frame + 3;
        
        // Unpack channels
        uint16_t channels[16];
        if (unpack_11bit_channels(channels, payload, 22)) {
            receiver->valid_frames++;
            
            // Update current channel data
            memcpy(receiver->current_channels.channels, channels, sizeof(channels));
            receiver->current_channels.frame_count = receiver->total_frames;
            receiver->current_channels.timestamp = get_timestamp_us();
            
            return 1; // Success
        }
    } else {
        printf("   🔍 Unsupported frame type: 0x%02x\n", frame_type);
    }
    
    return 0;
}

// Process UART data and extract complete CRSF frames
void process_uart_data(struct CRSFReceiver *receiver) {
    if (!receiver->uart.enabled) return;
    
    // Read available bytes from UART
    uint8_t read_buffer[128];
    int bytes_read = uart_receive_bytes(&receiver->uart, read_buffer, sizeof(read_buffer));
    
    if (bytes_read > 0) {
        // Add received bytes to buffer
        if (receiver->uart_buffer_pos + bytes_read <= UART_BUFFER_SIZE) {
            memcpy(receiver->uart_buffer + receiver->uart_buffer_pos, read_buffer, bytes_read);
            receiver->uart_buffer_pos += bytes_read;
        } else {
            // Buffer overflow, reset
            printf("⚠️  UART buffer overflow, resetting\n");
            receiver->uart_buffer_pos = 0;
            receiver->uart_errors++;
            return;
        }
        
        // Process complete frames in buffer
        size_t processed = 0;
        while (processed < receiver->uart_buffer_pos) {
            // Look for sync byte
            if (receiver->uart_buffer[processed] != CRSF_SYNC_BYTE) {
                processed++;
                continue;
            }
            
            // Check if we have enough data for frame size byte
            if (processed + 2 >= receiver->uart_buffer_pos) {
                break; // Need more data
            }
            
            // Get frame size from second byte
            uint8_t frame_size_byte = receiver->uart_buffer[processed + 1];
            size_t frame_length = frame_size_byte + 2; // Include sync and size bytes
            
            // Check if we have complete frame
            if (processed + frame_length > receiver->uart_buffer_pos) {
                break; // Need more data
            }
            
            // Process complete frame
            const uint8_t* frame = receiver->uart_buffer + processed;
            int result = process_crsf_frame(receiver, frame, frame_length);
            
            if (result > 0) {
                // Valid RC channels frame
                char timestamp[32];
                get_time_string(timestamp, sizeof(timestamp));
                
                printf("\n📨 Frame #%u [%s]\n", receiver->total_frames, timestamp);
                printf("   Type: 0x%02x, Size: %zu bytes, CRC: ✓ VALID\n", frame[2], frame_length);
                printf("   📦 RC_CHANNELS_PACKED packet\n");
                print_channels(&receiver->current_channels);
            } else if (result < 0) {
                // Invalid frame
                char timestamp[32];
                get_time_string(timestamp, sizeof(timestamp));
                
                printf("\n📨 Frame #%u [%s] - INVALID\n", receiver->total_frames, timestamp);
                printf("   Type: 0x%02x, Size: %zu bytes, CRC: ✗ INVALID\n", frame[2], frame_length);
            }
            
            processed += frame_length;
        }
        
        // Move remaining data to start of buffer
        if (processed > 0) {
            size_t remaining = receiver->uart_buffer_pos - processed;
            if (remaining > 0) {
                memmove(receiver->uart_buffer, receiver->uart_buffer + processed, remaining);
            }
            receiver->uart_buffer_pos = remaining;
        }
    } else if (bytes_read < 0 && errno != EAGAIN) {
        receiver->uart_errors++;
        printf("❌ UART read error: %s\n", strerror(errno));
    }
}

// Print channel values
void print_channels(const struct CRSFChannelData* channels) {
    const char* channel_names[] = {
        "ROLL", "PITCH", "THROTTLE", "YAW", "AUX1", "AUX2", "AUX3", "AUX4",
        "AUX5", "AUX6", "AUX7", "AUX8", "AUX9", "AUX10", "AUX11", "AUX12"
    };
    
    printf("   🎮 RC Channels:\n");
    
    // Display first 8 channels
    for (int i = 0; i < 8; i++) {
        uint16_t rc_value = crsf_to_rc(channels->channels[i]);
        const char* arrow = "";
        
        if (i < 4) {
            if (rc_value == 1500) arrow = "🔄 CENTER";
            else if (i == 0) arrow = (rc_value < 1500) ? "⬅️ LEFT" : "➡️ RIGHT";
            else if (i == 1) arrow = (rc_value < 1500) ? "⬇️ DOWN" : "⬆️ UP";
            else if (i == 2) arrow = (rc_value < 1500) ? "🔽 LOW" : "🔼 HIGH";
            else if (i == 3) arrow = (rc_value < 1500) ? "↩️ LEFT" : "↪️ RIGHT";
        }
        
        printf("     %-8s: %4dμs %s\n", channel_names[i], rc_value, arrow);
    }
    
    // Show compact format
    printf("   📊 Compact: ");
    for (int i = 0; i < 8; i++) {
        uint16_t rc_value = crsf_to_rc(channels->channels[i]);
        if (i < 4) {
            // Use first character of channel name
            printf("%c:%4d ", channel_names[i][0], rc_value);
        } else if (i == 4) {
            printf("| AUX: ");
        }
        if (i >= 4) {
            printf("%d:%4d ", i-3, rc_value);
        }
    }
    printf("\n");
}

// Print statistics
void print_statistics(const struct CRSFReceiver *receiver) {
    uint64_t current_time = get_timestamp_us();
    double runtime_seconds = (current_time - receiver->start_time) / 1000000.0;
    
    printf("\n📈 Receiver Statistics:\n");
    printf("   Total Frames: %u\n", receiver->total_frames);
    printf("   Valid Frames: %u\n", receiver->valid_frames);
    printf("   CRC Errors: %u\n", receiver->crc_errors);
    printf("   Sync Errors: %u\n", receiver->sync_errors);
    printf("   UART Errors: %u\n", receiver->uart_errors);
    
    if (receiver->uart.enabled) {
        printf("   UART Bytes: %d\n", receiver->uart.bytes_received);
    }
    
    if (runtime_seconds > 0) {
        double frames_per_second = receiver->total_frames / runtime_seconds;
        printf("   Frame Rate: %.2f Hz\n", frames_per_second);
        printf("   Runtime: %.1f seconds\n", runtime_seconds);
    }
    
    // Calculate success rate
    if (receiver->total_frames > 0) {
        double success_rate = (double)receiver->valid_frames / receiver->total_frames * 100.0;
        printf("   Success Rate: %.1f%%\n", success_rate);
    }
}

void receiver_run(struct CRSFReceiver *receiver) {
    printf("CRSF UART Receiver - Press Ctrl+C to exit\n");
    printf("UART Device: %s\n", receiver->uart.device);
    printf("Baud Rate: %d\n", receiver->uart.baud_rate);
    printf("Frame types supported: RC_CHANNELS_PACKED (0x16)\n");
    printf("--------------------------------------------------\n");
    
    uint32_t last_stats_time = 0;
    uint32_t last_frame_count = 0;
    
    while (running) {
        // Process UART data
        process_uart_data(receiver);
        
        // Print statistics every 5 seconds
        uint32_t current_time = (uint32_t)(get_timestamp_us() / 1000000);
        if (current_time - last_stats_time >= 5) {
            if (receiver->total_frames > last_frame_count) {
                printf("---\n");
            }
            print_statistics(receiver);
            last_stats_time = current_time;
            last_frame_count = receiver->total_frames;
        }
        
        // Small delay to prevent CPU spinning
        usleep(1000); // 1ms
    }
    
    // Print final statistics
    printf("\n==========================================\n");
    printf("Final Statistics:\n");
    print_statistics(receiver);
    printf("==========================================\n");
}

void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("Options:\n");
    printf("  --device DEVICE    UART device (default: /dev/ttyUSB0)\n");
    printf("  --baud RATE        Baud rate (default: 115200)\n");
    printf("  --help             Show this help message\n");
    printf("\nExamples:\n");
    printf("  %s --device /dev/ttyUSB0\n", program_name);
    printf("  %s --device /dev/ttyAMA0 --baud 115200\n", program_name);
    printf("  %s --device /dev/serial0 --baud 115200\n", program_name);
    printf("\nCommon UART devices:\n");
    printf("  /dev/ttyUSB0    - USB to serial adapter\n");
    printf("  /dev/ttyAMA0    - Raspberry Pi UART\n");
    printf("  /dev/serial0    - Raspberry Pi alternative\n");
    printf("  /dev/ttyS0      - PC serial port\n");
    printf("\nSupported baud rates:\n");
    printf("  9600, 19200, 38400, 57600, 115200, 230400, 460800,\n");
    printf("  500000, 921600, 1000000, 1152000, 1500000, 2000000,\n");
    printf("  2500000, 3000000, 3500000, 4000000\n");
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    
    char uart_device[64] = "/dev/ttyUSB0";
    int baud_rate = 115200; // Default to standard baud rate
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            strncpy(uart_device, argv[++i], sizeof(uart_device) - 1);
        } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baud_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Check if we have permission to access the UART device
    if (access(uart_device, R_OK | W_OK) != 0) {
        fprintf(stderr, "❌ Cannot access UART device %s: %s\n", uart_device, strerror(errno));
        fprintf(stderr, "   Try running with sudo or check device permissions\n");
        return 1;
    }
    
    struct CRSFReceiver receiver;
    receiver_init(&receiver, uart_device, baud_rate);
    
    if (!receiver.uart.enabled) {
        fprintf(stderr, "Failed to initialize receiver\n");
        return 1;
    }
    
    receiver_run(&receiver);
    receiver_cleanup(&receiver);
    
    printf("Receiver stopped\n");
    return 0;
}