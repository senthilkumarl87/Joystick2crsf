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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define MAX_FRAME_SIZE 64
#define UDP_BUFFER_SIZE 1024

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

struct UDPConfig {
    int sockfd;
    struct sockaddr_in client_addr;
    socklen_t client_len;
    int enabled;
    int port;
    int packets_received;
};

struct CRSFReceiver {
    // UDP configuration
    struct UDPConfig udp;
    
    // Statistics
    uint32_t total_frames;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t sync_errors;
    uint64_t start_time;
    
    // Current channel data
    struct CRSFChannelData current_channels;
};

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

// UDP functions
int udp_init(struct UDPConfig *udp, int port) {
    udp->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp->sockfd < 0) {
        perror("socket creation failed");
        return -1;
    }
    
    // Set socket options
    int reuse = 1;
    if (setsockopt(udp->sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("setsockopt failed");
        close(udp->sockfd);
        return -1;
    }
    
    // Set up server address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);
    
    if (bind(udp->sockfd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(udp->sockfd);
        return -1;
    }
    
    udp->client_len = sizeof(udp->client_addr);
    udp->enabled = 1;
    udp->port = port;
    udp->packets_received = 0;
    
    printf("UDP receiver initialized on port %d\n", port);
    return 0;
}

void udp_cleanup(struct UDPConfig *udp) {
    if (udp->sockfd >= 0) {
        close(udp->sockfd);
        udp->sockfd = -1;
    }
}

int udp_receive_packet(struct UDPConfig *udp, uint8_t* buffer, size_t buffer_size, char* client_ip, int* client_port) {
    if (!udp->enabled || udp->sockfd < 0) {
        return -1;
    }
    
    ssize_t received = recvfrom(udp->sockfd, buffer, buffer_size, MSG_DONTWAIT,
                               (struct sockaddr*)&udp->client_addr, &udp->client_len);
    
    if (received > 0) {
        udp->packets_received++;
        
        // Get client info
        if (client_ip) {
            inet_ntop(AF_INET, &udp->client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        }
        if (client_port) {
            *client_port = ntohs(udp->client_addr.sin_port);
        }
        
        return received;
    }
    
    return -1;
}

// Receiver initialization
void receiver_init(struct CRSFReceiver *receiver, int udp_port) {
    memset(receiver, 0, sizeof(*receiver));
    
    // Initialize UDP
    if (udp_init(&receiver->udp, udp_port) == 0) {
        printf("📡 UDP receiver enabled on port %d\n", receiver->udp.port);
    } else {
        fprintf(stderr, "❌ UDP initialization failed\n");
        receiver->udp.enabled = 0;
    }
    
    // Initialize channel data
    for (int i = 0; i < 16; i++) {
        receiver->current_channels.channels[i] = 992; // Center position
    }
    receiver->current_channels.num_channels = 16;
    receiver->current_channels.frame_count = 0;
    
    receiver->start_time = get_timestamp_us();
    
    printf("CRSF Receiver Initialized\n");
    printf("Frame types supported: RC_CHANNELS_PACKED (0x16)\n");
    printf("==========================================\n");
}

void receiver_cleanup(struct CRSFReceiver *receiver) {
    udp_cleanup(&receiver->udp);
}

// Process received CRSF frame
int process_crsf_frame(struct CRSFReceiver *receiver, const uint8_t* frame, size_t frame_len, 
                      const char* client_ip, int client_port) {
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

// Extract frame from UDP packet (with or without header)
size_t extract_frame_from_udp(const uint8_t* udp_data, size_t udp_len, uint8_t* frame_buffer) {
    if (udp_len < 3) return 0;
    
    // Check if it has 4-byte header
    if (udp_len >= 4) {
        uint32_t frame_size = ntohl(*(uint32_t*)udp_data);
        if (udp_len == frame_size + 4) {
            // Has header, extract frame
            memcpy(frame_buffer, udp_data + 4, frame_size);
            return frame_size;
        }
    }
    
    // No header or wrong size, assume raw frame
    if (udp_len <= MAX_FRAME_SIZE) {
        memcpy(frame_buffer, udp_data, udp_len);
        return udp_len;
    }
    
    return 0;
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

// Print frame information
void print_frame_info(const uint8_t* frame, size_t frame_len, const char* client_ip, 
                     int client_port, uint32_t frame_number, int is_valid) {
    char timestamp[32];
    get_time_string(timestamp, sizeof(timestamp));
    
    printf("\n📨 Frame #%u from %s:%d [%s]\n", 
           frame_number, client_ip, client_port, timestamp);
    
    printf("   Type: 0x%02x, Size: %zu bytes, CRC: %s\n", 
           frame[2], frame_len, is_valid ? "✓ VALID" : "✗ INVALID");
    
    if (frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        printf("   📦 RC_CHANNELS_PACKED packet\n");
    }
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
    
    if (receiver->udp.enabled) {
        printf("   UDP Packets: %d\n", receiver->udp.packets_received);
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

// Simple display mode (compact output)
void print_simple_display(const struct CRSFChannelData* channels, uint32_t frame_count) {
    char timestamp[32];
    get_time_string(timestamp, sizeof(timestamp));
    
    printf("[%s] Frame #%u: ", timestamp, frame_count);
    
    // Show first 4 control channels + 4 AUX channels
    for (int i = 0; i < 8; i++) {
        uint16_t rc_value = crsf_to_rc(channels->channels[i]);
        if (i < 4) {
            // Use single character labels for control channels
            const char* labels = "RPTY"; // Roll, Pitch, Throttle, Yaw
            printf("%c:%4d ", labels[i], rc_value);
        } else if (i == 4) {
            printf("| AUX: ");
        }
        if (i >= 4) {
            printf("%d:%4d ", i-3, rc_value);
        }
    }
    printf("\n");
}

void receiver_run(struct CRSFReceiver *receiver, int simple_mode) {
    printf("CRSF UDP Receiver - Press Ctrl+C to exit\n");
    printf("Mode: %s\n", simple_mode ? "Simple Display" : "Verbose Display");
    if (receiver->udp.enabled) {
        printf("Listening on UDP port %d\n", receiver->udp.port);
    }
    printf("--------------------------------------------------\n");
    
    uint8_t udp_buffer[UDP_BUFFER_SIZE];
    uint8_t frame_buffer[MAX_FRAME_SIZE];
    char client_ip[INET_ADDRSTRLEN];
    int client_port;
    
    uint32_t last_stats_time = 0;
    uint32_t last_frame_count = 0;
    
    while (running) {
        // Receive UDP packets
        if (receiver->udp.enabled) {
            ssize_t received = udp_receive_packet(&receiver->udp, udp_buffer, sizeof(udp_buffer), 
                                                 client_ip, &client_port);
            
            if (received > 0) {
                // Extract CRSF frame from UDP packet
                size_t frame_len = extract_frame_from_udp(udp_buffer, received, frame_buffer);
                
                if (frame_len > 0) {
                    // Process the frame
                    int result = process_crsf_frame(receiver, frame_buffer, frame_len, client_ip, client_port);
                    
                    if (result > 0) {
                        // Valid RC channels frame
                        if (simple_mode) {
                            print_simple_display(&receiver->current_channels, receiver->total_frames);
                        } else {
                            print_frame_info(frame_buffer, frame_len, client_ip, client_port, 
                                           receiver->total_frames, 1);
                            print_channels(&receiver->current_channels);
                        }
                    } else if (result == 0) {
                        // Other frame type
                        if (!simple_mode) {
                            print_frame_info(frame_buffer, frame_len, client_ip, client_port, 
                                           receiver->total_frames, 1);
                        }
                    } else {
                        // Invalid frame
                        if (!simple_mode) {
                            print_frame_info(frame_buffer, frame_len, client_ip, client_port, 
                                           receiver->total_frames, 0);
                        }
                    }
                }
            }
        }
        
        // Print statistics every 5 seconds
        uint32_t current_time = (uint32_t)(get_timestamp_us() / 1000000);
        if (current_time - last_stats_time >= 5) {
            if (!simple_mode && receiver->total_frames > last_frame_count) {
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
    printf("  --port PORT        UDP port to listen on (default: 8888)\n");
    printf("  --simple           Simple display mode (compact output)\n");
    printf("  --help             Show this help message\n");
    printf("\nExamples:\n");
    printf("  %s --port 8888\n", program_name);
    printf("  %s --port 9000 --simple\n", program_name);
    printf("  %s --simple\n", program_name);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    
    int udp_port = 8888;
    int simple_mode = 0;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            udp_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--simple") == 0) {
            simple_mode = 1;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    struct CRSFReceiver receiver;
    receiver_init(&receiver, udp_port);
    
    if (!receiver.udp.enabled) {
        fprintf(stderr, "Failed to initialize receiver\n");
        return 1;
    }
    
    receiver_run(&receiver, simple_mode);
    receiver_cleanup(&receiver);
    
    printf("Receiver stopped\n");
    return 0;
}