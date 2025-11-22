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

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_SIZE 26

volatile sig_atomic_t running = 1;

void signal_handler(int signal) {
    running = 0;
}

struct js_event {
    uint32_t time;
    int16_t value;
    uint8_t type;
    uint8_t number;
};

struct HighPrecisionTimer {
    struct timespec start_time;
    struct timespec last_frame;
    int64_t frame_interval_ns;
    int64_t accumulated_error;
    int64_t min_sleep_ns;
};

struct JoystickToCRSF {
    int fd;
    
    // RC values (1000-2000 range)
    int roll, pitch, yaw, throttle;
    int aux1, aux2, aux3, aux4;
    
    // Raw inputs
    int buttons[20];
    int axes[10];
    int hats_x[2];
    int hats_y[2];
    
    // Statistics
    int frames_generated;
    uint64_t total_jitter_ns;
    uint64_t max_jitter_ns;
    struct timespec start_time;
};

// Timing functions
void timespec_sub(struct timespec *result, const struct timespec *a, const struct timespec *b) {
    result->tv_sec = a->tv_sec - b->tv_sec;
    result->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (result->tv_nsec < 0) {
        result->tv_sec--;
        result->tv_nsec += 1000000000;
    }
}

int64_t timespec_to_ns(const struct timespec *ts) {
    return ((int64_t)ts->tv_sec * 1000000000) + ts->tv_nsec;
}

void ns_to_timespec(struct timespec *ts, int64_t ns) {
    ts->tv_sec = ns / 1000000000;
    ts->tv_nsec = ns % 1000000000;
}

void timer_init(struct HighPrecisionTimer *timer, int frequency_hz) {
    clock_gettime(CLOCK_MONOTONIC, &timer->start_time);
    timer->last_frame = timer->start_time;
    timer->frame_interval_ns = 1000000000 / frequency_hz;
    timer->accumulated_error = 0;
    timer->min_sleep_ns = 1000000; // 1ms
}

void timer_wait_next_frame(struct HighPrecisionTimer *timer) {
    struct timespec now, elapsed;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    timespec_sub(&elapsed, &now, &timer->last_frame);
    int64_t elapsed_ns = timespec_to_ns(&elapsed);
    int64_t sleep_ns = timer->frame_interval_ns - elapsed_ns - timer->accumulated_error;
    
    // Apply error correction
    if (llabs(timer->accumulated_error) > timer->frame_interval_ns / 2) {
        timer->accumulated_error = 0;
    }
    
    if (sleep_ns > timer->min_sleep_ns) {
        // Sleep for most of the time
        struct timespec sleep_time;
        ns_to_timespec(&sleep_time, sleep_ns - timer->min_sleep_ns);
        nanosleep(&sleep_time, NULL);
        
        // Busy-wait for remaining time
        struct timespec busy_start;
        clock_gettime(CLOCK_MONOTONIC, &busy_start);
        int64_t busy_elapsed = 0;
        while (busy_elapsed < timer->min_sleep_ns) {
            struct timespec busy_now;
            clock_gettime(CLOCK_MONOTONIC, &busy_now);
            timespec_sub(&elapsed, &busy_now, &busy_start);
            busy_elapsed = timespec_to_ns(&elapsed);
        }
    } else if (sleep_ns > 0) {
        // Short delay
        usleep(sleep_ns / 1000);
    }
    
    // Calculate timing error for next frame
    struct timespec actual_now;
    clock_gettime(CLOCK_MONOTONIC, &actual_now);
    timespec_sub(&elapsed, &actual_now, &timer->last_frame);
    int64_t actual_elapsed_ns = timespec_to_ns(&elapsed);
    timer->accumulated_error += (actual_elapsed_ns - timer->frame_interval_ns);
    
    timer->last_frame = actual_now;
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

// Convert RC value (1000-2000) to CRSF format (0-1984)
uint16_t rc_to_crsf(int rc_value) {
    // Limit to valid range
    if (rc_value < 1000) rc_value = 1000;
    if (rc_value > 2000) rc_value = 2000;
    
    // Convert 1000-2000 to 0-1984 using integer math only
    return (uint16_t)((rc_value - 1000) * 1984 / 1000);
}

// Generate CRSF RC channels packet with proper CRC
void generate_crsf_frame(uint8_t* frame, size_t* frame_len, const uint16_t* channels, int num_channels) {
    // Frame structure: [SYNC_BYTE][FRAME_SIZE][FRAME_TYPE][PAYLOAD][CRC]
    uint8_t payload[22] = {0};
    
    // Pack 11-bit channels into bytes
    for (int i = 0; i < 16; i++) {
        uint16_t channel_value = (i < num_channels) ? channels[i] : 992; // Center position
        
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
    int frame_index = 0;
    frame[frame_index++] = CRSF_SYNC_BYTE;                    // Sync byte
    frame[frame_index++] = 24;                                // Frame size: type(1) + payload(22) + CRC(1) = 24
    frame[frame_index++] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED; // Frame type
    
    // Add payload
    for (int i = 0; i < 22; i++) {
        frame[frame_index++] = payload[i];
    }
    
    // Calculate proper CRC (from frame[2] to frame[-1])
    uint8_t crc = crc8_data(frame + 2, frame_index - 2);
    frame[frame_index++] = crc;
    
    *frame_len = frame_index;
}

// Print CRSF frame in hex format
void print_crsf_frame(const uint8_t* frame, size_t frame_len) {
    printf("CRSF Frame: ");
    for (size_t i = 0; i < frame_len; i++) {
        printf("%02x ", frame[i]);
    }
    printf("\n");
    
    // Also print CRC calculation details
    if (frame_len >= 4) {
        // Calculate what the CRC should be
        uint8_t calculated_crc = crc8_data(frame + 2, frame_len - 3); // From type byte to before CRC
        uint8_t frame_crc = frame[frame_len - 1];
        
        printf("CRC: data=");
        for (size_t i = 2; i < frame_len - 1; i++) {
            printf("%02x ", frame[i]);
        }
        printf("-> calculated=0x%02x, frame=0x%02x", calculated_crc, frame_crc);
        if (calculated_crc == frame_crc) {
            printf(" ✓ VALID\n");
        } else {
            printf(" ✗ INVALID\n");
        }
    }
}

// Print channel values
void print_channels(const uint16_t* channels, int num_channels) {
    const char* channel_names[] = {
        "ROLL", "PITCH", "THROTTLE", "YAW", "AUX1", "AUX2", "AUX3", "AUX4",
        "AUX5", "AUX6", "AUX7", "AUX8", "AUX9", "AUX10", "AUX11", "AUX12"
    };
    
    printf("Channels: ");
    for (int i = 0; i < 8 && i < num_channels; i++) {
        // Convert back to RC value for display
        int rc_value = (channels[i] * 1000) / 1984 + 1000;
        printf("%s:%d ", channel_names[i], rc_value);
    }
    printf("\n");
    
    printf("Raw CRSF: ");
    for (int i = 0; i < 8 && i < num_channels; i++) {
        printf("%s:%d ", channel_names[i], channels[i]);
    }
    printf("\n");
}

// Joystick functions
int normalized_to_rc(int normalized_value) {
    // Convert -100 to 100 range to 1000-2000
    int rc_value = 1500 + (normalized_value * 5);
    
    // Limit to valid range
    if (rc_value < 1000) rc_value = 1000;
    if (rc_value > 2000) rc_value = 2000;
    
    return rc_value;
}

void joystick_init(struct JoystickToCRSF *converter, const char* device) {
    converter->fd = open(device, O_RDONLY);
    if (converter->fd == -1) {
        fprintf(stderr, "Could not open joystick device: %s\n", device);
        exit(1);
    }
    
    // Initialize all values to center
    converter->roll = converter->pitch = converter->yaw = converter->throttle = 1500;
    converter->aux1 = converter->aux2 = converter->aux3 = converter->aux4 = 1500;
    
    // Initialize arrays
    memset(converter->buttons, 0, sizeof(converter->buttons));
    memset(converter->axes, 0, sizeof(converter->axes));
    memset(converter->hats_x, 0, sizeof(converter->hats_x));
    memset(converter->hats_y, 0, sizeof(converter->hats_y));
    
    // Set non-blocking mode
    int flags = fcntl(converter->fd, F_GETFL, 0);
    fcntl(converter->fd, F_SETFL, flags | O_NONBLOCK);
    
    clock_gettime(CLOCK_MONOTONIC, &converter->start_time);
    converter->frames_generated = 0;
    converter->total_jitter_ns = 0;
    converter->max_jitter_ns = 0;
    
    printf("Joystick to CRSF Converter Initialized\n");
    printf("High Precision Timer: 50Hz (20ms intervals)\n");
    printf("Axis Mapping: \n");
    printf("  Axis 0: YAW | Axis 1: THROTTLE | Axis 3: ROLL | Axis 4: PITCH\n");
    printf("==========================================\n");
}

void joystick_cleanup(struct JoystickToCRSF *converter) {
    if (converter->fd != -1) close(converter->fd);
}

void update_controls(struct JoystickToCRSF *converter) {
    // Convert normalized values to RC values (1000-2000)
    int yaw_norm = converter->axes[0] / 327.67;
    int throttle_norm = -converter->axes[1] / 327.67;  // Inverted
    int roll_norm = converter->axes[3] / 327.67;
    int pitch_norm = converter->axes[4] / 327.67;
    
    converter->yaw = normalized_to_rc(yaw_norm);
    converter->throttle = normalized_to_rc(throttle_norm);
    converter->roll = normalized_to_rc(roll_norm);
    converter->pitch = normalized_to_rc(pitch_norm);
}

void update_aux_channels(struct JoystickToCRSF *converter) {
    // AUX1: Flight mode from buttons 0-2
    if (converter->buttons[0]) converter->aux1 = 1000;        // Manual
    else if (converter->buttons[1]) converter->aux1 = 1500;   // Stabilize
    else if (converter->buttons[2]) converter->aux1 = 2000;   // Auto
    else converter->aux1 = 1500;
    
    // AUX2: From buttons 3-4
    if (converter->buttons[3]) converter->aux2 = 2000;
    else if (converter->buttons[4]) converter->aux2 = 1000;
    else converter->aux2 = 1500;
    
    // AUX3: From trigger (axis 2) if available
    int trigger_norm = converter->axes[2] / 327.67;
    converter->aux3 = normalized_to_rc(trigger_norm);
    
    // AUX4: From hat switch
    if (converter->hats_y[0] == 1) converter->aux4 = 2000;   // Hat up
    else if (converter->hats_y[0] == -1) converter->aux4 = 1000; // Hat down
    else converter->aux4 = 1500;
}

void process_event(struct JoystickToCRSF *converter, const struct js_event* event) {
    switch (event->type) {
        case 0x01:  // Button event
            if (event->number < 20) {
                converter->buttons[event->number] = (event->value == 1);
                update_aux_channels(converter);
            }
            break;
            
        case 0x02:  // Axis event
            if (event->number < 10) {
                converter->axes[event->number] = event->value;
                update_controls(converter);
            }
            break;
            
        case 0x03:  // Hat event
            if (event->number < 2) {
                converter->hats_x[event->number] = event->value & 0xFF;
                converter->hats_y[event->number] = (event->value >> 8) & 0xFF;
                update_aux_channels(converter);
            }
            break;
    }
}

void get_crsf_channels(struct JoystickToCRSF *converter, uint16_t* channels, int num_channels) {
    // Initialize with center position
    for (int i = 0; i < num_channels; i++) {
        channels[i] = 992;
    }
    
    // Map RC values to CRSF channels
    channels[0] = rc_to_crsf(converter->roll);     // ROLL
    channels[1] = rc_to_crsf(converter->pitch);    // PITCH
    channels[2] = rc_to_crsf(converter->throttle); // THROTTLE
    channels[3] = rc_to_crsf(converter->yaw);      // YAW
    channels[4] = rc_to_crsf(converter->aux1);     // AUX1
    channels[5] = rc_to_crsf(converter->aux2);     // AUX2
    channels[6] = rc_to_crsf(converter->aux3);     // AUX3
    channels[7] = rc_to_crsf(converter->aux4);     // AUX4
}

void update_timing_stats(struct JoystickToCRSF *converter, int64_t jitter_ns) {
    converter->total_jitter_ns += llabs(jitter_ns);
    if (llabs(jitter_ns) > converter->max_jitter_ns) {
        converter->max_jitter_ns = llabs(jitter_ns);
    }
}

void print_status(struct JoystickToCRSF *converter, double current_jitter_ns) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    struct timespec runtime_ts;
    timespec_sub(&runtime_ts, &now, &converter->start_time);
    int runtime_sec = runtime_ts.tv_sec;
    
    // Main flight controls
    printf("RC: R:%4d P:%4d Y:%4d T:%4d", 
           converter->roll, converter->pitch, converter->yaw, converter->throttle);
    
    // AUX channels
    printf(" | AUX: %d %d %d %d", 
           converter->aux1, converter->aux2, converter->aux3, converter->aux4);
    
    // Timing statistics
    double avg_jitter = converter->frames_generated > 0 ? 
        (converter->total_jitter_ns / converter->frames_generated) / 1000000.0 : 0.0;
    printf(" | Timing: Jitter=%.2fms Avg=%.2fms Max=%.2fms", 
           current_jitter_ns / 1000000.0, avg_jitter, converter->max_jitter_ns / 1000000.0);
    
    // Frame statistics
    printf(" | Frames: %d", converter->frames_generated);
    printf(" | Runtime: %ds", runtime_sec);
    
    printf("\n");
}

void read_joystick_events(struct JoystickToCRSF *converter) {
    struct js_event event;
    ssize_t bytes;
    
    // Read all available joystick events
    while ((bytes = read(converter->fd, &event, sizeof(event))) > 0) {
        if (bytes == sizeof(event)) {
            process_event(converter, &event);
        }
    }
}

void joystick_run(struct JoystickToCRSF *converter) {
    printf("Joystick to CRSF Converter - Press Ctrl+C to exit\n");
    printf("High Precision Mode: 50Hz CRSF frames with <1ms jitter\n");
    printf("--------------------------------------------------\n");
    
    struct HighPrecisionTimer timer;
    timer_init(&timer, 50); // 50Hz CRSF
    
    uint16_t channels[16];
    uint8_t crsf_frame[64];
    size_t frame_len;
    
    while (running) {
        // Read all available joystick events (non-blocking)
        read_joystick_events(converter);
        
        // Generate CRSF frame at precise 50Hz intervals
        struct timespec frame_start, frame_end, processing_time;
        clock_gettime(CLOCK_MONOTONIC, &frame_start);
        
        // Get current channel values
        get_crsf_channels(converter, channels, 16);
        
        // Generate CRSF frame with proper CRC
        generate_crsf_frame(crsf_frame, &frame_len, channels, 16);
        
        // Calculate timing jitter
        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        timespec_sub(&processing_time, &frame_end, &frame_start);
        int64_t jitter_ns = timespec_to_ns(&processing_time);
        
        // Update timing statistics
        update_timing_stats(converter, jitter_ns);
        
        // Print detailed information every 50 frames (1 second) to reduce console spam
        if (converter->frames_generated % 50 == 0) {
            time_t now = time(NULL);
            struct tm* local_time = localtime(&now);
            printf("\n[%02d:%02d:%02d] Frame #%d\n", 
                   local_time->tm_hour, local_time->tm_min, local_time->tm_sec,
                   converter->frames_generated + 1);
            
            // Print RC status
            print_status(converter, jitter_ns);
            
            // Print channel information
            print_channels(channels, 16);
            
            // Print CRSF frame (first frame and every 100th frame thereafter)
            if (converter->frames_generated == 0 || converter->frames_generated % 100 == 0) {
                print_crsf_frame(crsf_frame, frame_len);
            }
            
            printf("---\n");
        }
        
        converter->frames_generated++;
        
        // Wait for next frame with high precision
        timer_wait_next_frame(&timer);
    }
    
    // Print final statistics
    struct timespec end_time;
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    struct timespec total_runtime;
    timespec_sub(&total_runtime, &end_time, &converter->start_time);
    double avg_jitter = converter->frames_generated > 0 ? 
        (converter->total_jitter_ns / converter->frames_generated) / 1000000.0 : 0.0;
    double frames_per_second = converter->frames_generated / (double)total_runtime.tv_sec;
    
    printf("\n==========================================\n");
    printf("Final Statistics:\n");
    printf("  Total Frames: %d\n", converter->frames_generated);
    printf("  Total Runtime: %ld seconds\n", total_runtime.tv_sec);
    printf("  Actual Frame Rate: %.2f Hz\n", frames_per_second);
    printf("  Average Jitter: %.2f ms\n", avg_jitter);
    printf("  Maximum Jitter: %.2f ms\n", converter->max_jitter_ns / 1000000.0);
    printf("==========================================\n");
}

int main() {
    signal(SIGINT, signal_handler);
    
    // Set high priority for better timing (requires root)
    if (nice(-20) == -1) {
        printf("Note: Could not set high process priority (run with sudo for better timing)\n");
    }
    
    struct JoystickToCRSF converter;
    joystick_init(&converter, "/dev/input/js0");
    joystick_run(&converter);
    joystick_cleanup(&converter);
    
    printf("Exiting...\n");
    return 0;
}