#!/usr/bin/env python3
import socket
import struct
import time
import threading
from datetime import datetime
from enum import IntEnum

# CRSF Constants and parsing from reference code
CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
    crc = crc ^ a
    for ii in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    return b - 256 if b >= 128 else b

def unpack_11bit_channels(payload):
    """Unpack 16 channels of 11-bit data from 22-byte payload - CORRECTED VERSION"""
    channels = []
    
    if len(payload) < 22:
        return channels
    
    # Process all 16 channels
    for channel in range(16):
        # Calculate bit position for this channel
        bit_pos = channel * 11
        byte_pos = bit_pos // 8
        bit_in_byte = bit_pos % 8
        
        # Ensure we have enough bytes
        if byte_pos + 2 >= len(payload):
            channels.append(992)  # Default center value
            continue
            
        # Extract 11 bits across byte boundaries
        if bit_in_byte <= 5:
            # Bits fit within 2 bytes
            low_byte = payload[byte_pos]
            high_byte = payload[byte_pos + 1]
            
            # Extract 11 bits
            value = ((low_byte >> bit_in_byte) | 
                    (high_byte << (8 - bit_in_byte))) & 0x7FF
        else:
            # Bits span 3 bytes
            bit_shift = bit_in_byte - 5
            low_byte = payload[byte_pos]
            mid_byte = payload[byte_pos + 1]
            high_byte = payload[byte_pos + 2]
            
            # Extract 11 bits across 3 bytes
            value = ((low_byte >> bit_in_byte) |
                    (mid_byte << (8 - bit_in_byte)) |
                    (high_byte << (16 - bit_in_byte))) & 0x7FF
        
        channels.append(value)
    
    return channels

def crsf_to_rc(crsf_value):
    """Convert CRSF value (0-1984) to RC value (1000-2000μs)"""
    return int((crsf_value / 1.984) + 1000)

def get_control_arrow(channel_index, rc_value):
    """Get arrow representation for control channels"""
    if rc_value == 1500:
        return "🔄 CENTER"
    elif channel_index == 0:  # ROLL
        return "⬅️ LEFT" if rc_value < 1500 else "➡️ RIGHT"
    elif channel_index == 1:  # PITCH
        return "⬇️ DOWN" if rc_value < 1500 else "⬆️ UP"
    elif channel_index == 2:  # THROTTLE
        return "🔽 LOW" if rc_value < 1500 else "🔼 HIGH"
    elif channel_index == 3:  # YAW
        return "↩️ LEFT" if rc_value < 1500 else "↪️ RIGHT"
    return ""

def get_aux_state(rc_value):
    """Get state description for AUX channels"""
    if rc_value < 1200:
        return "LOW"
    elif rc_value < 1400:
        return "MID-LOW"
    elif rc_value < 1600:
        return "MID"
    elif rc_value < 1800:
        return "MID-HIGH"
    else:
        return "HIGH"

class CRSFUDPReceiver:
    def __init__(self, host='0.0.0.0', port=8888, buffer_size=1024):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None
        self.running = False
        self.receive_count = 0
        self.error_count = 0
        self.last_frame_time = None
        
    def start_receiver(self):
        """Start the UDP receiver"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            
            self.running = True
            print(f"🚀 CRSF UDP Receiver started")
            print(f"📡 Listening on {self.host}:{self.port}")
            print("Press Ctrl+C to stop\n")
            
            # Start statistics thread
            stats_thread = threading.Thread(target=self._print_statistics, daemon=True)
            stats_thread.start()
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(self.buffer_size)
                    if data:
                        self.receive_count += 1
                        self.last_frame_time = time.time()
                        self.process_frame(data, addr)
                        
                except socket.timeout:
                    continue
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self.error_count += 1
                    print(f"❌ Receive error: {e}")
                    
        except Exception as e:
            print(f"❌ Failed to start receiver: {e}")
        finally:
            self.stop()
            
    def process_frame(self, data, addr):
        """Process received UDP packet containing CRSF frame"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        try:
            # Extract frame data (skip 4-byte header if present)
            if len(data) >= 4:
                # Try to parse with header (big-endian 4-byte size)
                frame_size = struct.unpack('>I', data[:4])[0]
                if len(data) == frame_size + 4:
                    frame_data = data[4:]
                    self._process_crsf_frame(frame_data, addr, timestamp)
                    return
            
            # If no header or wrong size, try as raw frame
            if len(data) >= 3:
                self._process_crsf_frame(data, addr, timestamp)
            else:
                print(f"❓ Unknown data format from {addr}")
                    
        except Exception as e:
            self.error_count += 1
            print(f"❌ Frame processing error: {e}")
            
    def _process_crsf_frame(self, frame_data, addr, timestamp):
        """Process CRSF frame using reference parsing method"""
        frame_bytes = bytearray(frame_data)
        
        # Basic frame validation
        if len(frame_bytes) < 4:
            print(f"   ❌ Frame too short: {len(frame_bytes)} bytes")
            return
            
        sync_byte = frame_bytes[0]
        frame_size_byte = frame_bytes[1]
        frame_type = frame_bytes[2]
        
        # Validate sync byte
        if sync_byte != CRSF_SYNC:
            print(f"   ❌ Invalid sync byte: 0x{sync_byte:02x} (expected 0x{CRSF_SYNC:02x})")
            return
            
        # Calculate expected length
        expected_len = frame_size_byte + 2
        
        if len(frame_bytes) != expected_len:
            print(f"   ⚠️  Frame size: got {len(frame_bytes)}, expected {expected_len} (size_byte={frame_size_byte})")
        
        # Validate CRC
        crc_valid = False
        if len(frame_bytes) == expected_len:
            if crsf_validate_frame(frame_bytes):
                crc_valid = True
            else:
                print(f"   ❌ CRC error")
                return
        else:
            print(f"   ⚠️  Skipping CRC check due to size mismatch")
            crc_valid = True
        
        print(f"\n📨 Frame #{self.receive_count} from {addr[0]}:{addr[1]} [{timestamp}]")
        print(f"   Type: 0x{frame_type:02x}, Size: {len(frame_bytes)} bytes, CRC: {'✓ VALID' if crc_valid else '⚠️ UNCHECKED'}")
        
        # Handle RC channels packet
        if frame_type == PacketsTypes.RC_CHANNELS_PACKED:
            print(f"   📦 RC_CHANNELS_PACKED packet")
            self._handle_rc_channels_packet(frame_bytes, crc_valid)
        else:
            try:
                packet_name = PacketsTypes(frame_type).name
                print(f"   🔍 {packet_name} packet (not implemented)")
            except ValueError:
                print(f"   🔍 Unknown packet type: 0x{frame_type:02x}")
    
    def _handle_rc_channels_packet(self, frame_bytes, crc_valid):
        """Handle RC channels packed packet"""
        if len(frame_bytes) < 26:
            print("   ❌ Invalid RC channels packet (too short)")
            return
        
        # Extract payload
        payload = frame_bytes[3:25]
        
        if len(payload) < 22:
            print(f"   ❌ Invalid payload length: {len(payload)} bytes (expected 22)")
            return
        
        # Unpack 11-bit channels
        channels = unpack_11bit_channels(payload)
        
        print(f"   🔧 Unpacked {len(channels)} channels")
        
        if len(channels) < 16:
            print(f"   ⚠️  Only got {len(channels)} channels (expected 16), but continuing...")
        
        # Channel names
        channel_names = [
            "ROLL", "PITCH", "THROTTLE", "YAW", 
            "AUX1", "AUX2", "AUX3", "AUX4",
            "AUX5", "AUX6", "AUX7", "AUX8",
            "AUX9", "AUX10", "AUX11", "AUX12"
        ]
        
        print("   🎮 RC Channels:")
        
        # Display channels
        for i in range(min(len(channels), 8)):
            rc_value = crsf_to_rc(channels[i])
            
            if i < 4:
                arrow = get_control_arrow(i, rc_value)
                print(f"     {channel_names[i]}: {rc_value:4d}μs {arrow}")
            else:
                state = get_aux_state(rc_value)
                print(f"     {channel_names[i]}: {rc_value:4d}μs [{state}]")
        
        # Show compact format
        if len(channels) >= 8:
            rc_values = [crsf_to_rc(ch) for ch in channels[:8]]
            print(f"   📊 Compact: R:{rc_values[0]:4d} P:{rc_values[1]:4d} T:{rc_values[2]:4d} Y:{rc_values[3]:4d} "
                  f"| AUX: {rc_values[4]:4d} {rc_values[5]:4d} {rc_values[6]:4d} {rc_values[7]:4d}")
    
    def _print_statistics(self):
        """Print periodic statistics"""
        while self.running:
            time.sleep(5)
            if self.receive_count > 0:
                print(f"\n📈 Statistics: {self.receive_count} frames, {self.error_count} errors")
                if self.last_frame_time:
                    time_since_last = time.time() - self.last_frame_time
                    print(f"   Last frame: {time_since_last:.1f}s ago")
                    
    def stop(self):
        """Stop the receiver"""
        self.running = False
        if self.sock:
            self.sock.close()
        print(f"\n🛑 Receiver stopped. Total frames: {self.receive_count}")

# Simple receiver for RC channels only
class SimpleRCReceiver:
    def __init__(self, host='0.0.0.0', port=8888):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        
    def start_receiver(self):
        """Simple receiver that only shows RC values"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            
            self.running = True
            print(f"Simple RC Receiver on {self.host}:{self.port}")
            print("Waiting for RC data...\n")
            
            frame_count = 0
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    if data:
                        frame_count += 1
                        self.process_simple(data, frame_count)
                        
                except socket.timeout:
                    continue
                except KeyboardInterrupt:
                    break
                    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self.sock:
                self.sock.close()
    
    def process_simple(self, data, frame_count):
        """Simple processing that only extracts RC values"""
        try:
            # Extract frame data
            frame_data = data
            if len(data) >= 4:
                frame_size = struct.unpack('>I', data[:4])[0]
                if len(data) == frame_size + 4:
                    frame_data = data[4:]
            
            # Basic validation
            if (len(frame_data) >= 26 and 
                frame_data[0] == CRSF_SYNC and 
                frame_data[2] == 0x16):
                
                # Extract payload
                payload = frame_data[3:25]
                if len(payload) >= 22:
                    channels = unpack_11bit_channels(payload)
                    
                    if len(channels) >= 8:
                        # Convert to RC values and display
                        rc_values = [crsf_to_rc(ch) for ch in channels[:8]]
                        
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"[{timestamp}] Frame #{frame_count}: "
                              f"R:{rc_values[0]:4d} P:{rc_values[1]:4d} T:{rc_values[2]:4d} Y:{rc_values[3]:4d} "
                              f"| AUX: {rc_values[4]:4d} {rc_values[5]:4d} {rc_values[6]:4d} {rc_values[7]:4d}")
        
        except Exception as e:
            pass

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='CRSF UDP Receiver')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8888, help='Port to listen on')
    parser.add_argument('--simple', action='store_true', help='Use simple receiver (RC values only)')
    
    args = parser.parse_args()
    
    if args.simple:
        receiver = SimpleRCReceiver(args.host, args.port)
        receiver.start_receiver()
    else:
        receiver = CRSFUDPReceiver(args.host, args.port)
        receiver.start_receiver()