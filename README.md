# Joystick2CRSF

A versatile bridge that converts joystick inputs to the Crossfire (CRSF) RC protocol, enabling control of RC vehicles using standard USB joysticks. The project also includes support for MAVLink telemetry and UDP communication.

## Features

- Convert USB joystick inputs to CRSF protocol
- Support for 16 RC channels (4 primary + 12 auxiliary)
- MAVLink telemetry integration
- UDP bridge for network connectivity
- Support for multiple input devices
- Configurable channel mapping
- Real-time status monitoring

## Prerequisites

- Linux-based system (tested on Ubuntu/Raspberry Pi OS)
- C++17 compatible compiler
- Python 3.6+ (for UDP tools)
- USB joystick/gamepad
- CRSF-compatible receiver

## Installation

### Dependencies

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y build-essential python3-pip

# Install Python dependencies
pip3 install pyserial
```

### Building from Source

```bash
# Clone the repository
git clone https://github.com/yourusername/Joystick2crsf.git
cd Joystick2crsf

# Build the main application
make

# Or build manually
g++ -std=c++17 joystick_to_crsf.cpp -o joystick_to_crsf
g++ -std=c++17 crsf_mavlink_rc_bridge_with_binding.cpp -o crsf_mavlink_bridge -pthread
```

## Usage

### Basic Joystick to CRSF

```bash
# Run with default settings (requires root for joystick access)
sudo ./joystick_to_crsf

# Specify custom joystick device
sudo ./joystick_to_crsf /dev/input/js1
```

### MAVLink Bridge

```bash
# Start the MAVLink to CRSF bridge
./crsf_mavlink_bridge /dev/ttyUSB0 115200
```

### UDP Receiver

```bash
# Start the UDP receiver (default port 8888)
python3 crsf_udp_receiver.py

# Specify custom host and port
python3 crsf_udp_receiver.py --host 0.0.0.0 --port 9000
```

## Configuration

### Channel Mapping

Edit the `JoystickToCRSF` class in `joystick_to_crsf.cpp` to modify the default channel mapping:

```cpp
// Default channel mapping
channels[CRSFGenerator::CH_ROLL] = CRSFGenerator::rc_to_crsf(roll);
channels[CRSFGenerator::CH_PITCH] = CRSFGenerator::rc_to_crsf(pitch);
channels[CRSFGenerator::CH_THROTTLE] = CRSFGenerator::rc_to_crsf(throttle);
channels[CRSFGenerator::CH_YAW] = CRSFGenerator::rc_to_crsf(yaw);
// ...
```

### Button and Axis Configuration

Modify the `update_controls()` and `update_aux_channels()` methods in `joystick_to_crsf.cpp` to customize button and axis mappings.

## Project Structure

```
Joystick2CRSF/
├── crsf_mavlink_rc_bridge_with_binding.cpp  # MAVLink to CRSF bridge
├── crsf_udp_receiver.py                    # UDP receiver for CRSF frames
├── joystick_to_crsf.cpp                    # Main joystick to CRSF converter
├── joystick_reader.cpp                     # Basic joystick input reader
├── common.h                               # Common definitions and structures
└── README.md                              # This file
```

## Protocol Support

### CRSF Protocol
- RC channels (packed 11-bit format)
- Device information
- Binding
- Telemetry (GPS, battery, attitude)

### MAVLink Protocol
- Heartbeat
- RC channels
- Manual control
- Attitude
- GPS raw data

## Troubleshooting

### Joystick Not Detected
- Ensure the joystick is properly connected
- Check device permissions: `ls -l /dev/input/js*`
- Try running with `sudo` if permission denied

### No Output
- Verify CRSF receiver is connected and powered
- Check baud rate settings (default: 115200)
- Ensure proper serial port is specified

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- TBS Crossfire protocol documentation
- MAVLink development team
- OpenTX/EdgeTX communities for protocol references
