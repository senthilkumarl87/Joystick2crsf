#!/usr/bin/env python3
import socket
import struct
import time
import threading
from datetime import datetime

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
            self.sock.settimeout(1.0)  # 1 second timeout for clean shutdown
            
            self.running = True
            print(f"🚀 CRSF UDP Receiver started")
            print(f"📡 Listening on {self.host}:{self.port}")
            print(f"📊 Buffer size: {self.buffer_size} bytes")
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
            # Check if packet has header (first 4 bytes are frame size)
            if len(data) >= 4:
                # Try to parse with header
                frame_size = struct.unpack('>I', data[:4])[0]  # Big-endian 4-byte size
                
                if len(data) == frame_size + 4:  # Header + frame data
                    frame_data = data[4:]
                    self._process_crsf_frame(frame_data, addr, timestamp, "With Header")
                    return
                    
            # If no header or wrong size, try to parse as raw CRSF frame
            if len(data) >= 3:  # Minimum CRSF frame has sync+size+type
                self._process_crsf_frame(data, addr, timestamp, "Raw Frame")
            else:
                print(f"❓ Unknown data format from {addr}")
                    
        except Exception as e:
            self.error_count += 1
            print(f"❌ Frame processing error: {e}")
            
    def _process_crsf_frame(self, frame_data, addr, timestamp, frame_type):
        """Process actual CRSF frame data"""
        frame_bytes = list(frame_data)
        
        print(f"\n📨 Frame #{self.receive_count} from {addr[0]}:{addr[1]} [{timestamp}]")
        print(f"   Type: {frame_type}, Size: {len(frame_bytes)} bytes")
        print(f"   Hex: {''.join(f'{b:02x}' for b in frame_bytes)}")
        
        # Basic CRSF frame parsing
        if len(frame_bytes) >= 3:
            sync_byte = frame_bytes[0]
            frame_size = frame_bytes[1]
            frame_type_byte = frame_bytes[2]
            crc_byte = frame_bytes[-1] if len(frame_bytes) > 3 else 0
            
            print(f"   Sync: 0x{sync_byte:02x}, Size: {frame_size}, Type: 0x{frame_type_byte:02x}, CRC: 0x{crc_byte:02x}")
            
            # Parse specific frame types
            if frame_type_byte == 0x16:  # RC channels packet
                print("   🎮 RC Channels packet detected")
                self._parse_rc_channels_packet(frame_bytes[3:-1])  # Skip header and CRC
            elif frame_type_byte == 0x21:  # Link statistics
                print("   📊 Link Statistics packet")
            else:
                print(f"   🔍 Unknown frame type: 0x{frame_type_byte:02x}")
    
    def _parse_rc_channels_packet(self, payload):
        """Parse RC channels from CRSF payload"""
        if len(payload) < 22:
            print("   ❌ Invalid RC channels payload (too short)")
            return
            
        print("   🎯 RC Channels (16 channels):")
        
        # Extract 11-bit channels from payload
        channels = self._unpack_11bit_channels(payload)
        
        # Channel names
        channel_names = [
            "ROLL", "PITCH", "THROTTLE", "YAW", 
            "AUX1", "AUX2", "AUX3", "AUX4",
            "AUX5", "AUX6", "AUX7", "AUX8",
            "AUX9", "AUX10", "AUX11", "AUX12"
        ]
        
        # Convert CRSF values (0-1984) back to RC values (1000-2000μs)
        for i in range(min(12, len(channels))):  # Show first 12 channels
            crsf_value = channels[i]
            rc_value = self._crsf_to_rc(crsf_value)
            
            # Format output based on channel type
            if i < 4:  # Main flight controls
                arrow = self._get_control_arrow(i, rc_value)
                print(f"     {channel_names[i]}: {rc_value:4d}μs {arrow}")
            else:  # AUX channels
                state = self._get_aux_state(rc_value)
                print(f"     {channel_names[i]}: {rc_value:4d}μs [{state}]")
    
    def _unpack_11bit_channels(self, payload):
        """Unpack 16 channels of 11-bit data from 22-byte payload"""
        channels = []
        
        for i in range(16):
            # Calculate byte position for this channel
            bit_pos = i * 11
            byte_pos = bit_pos // 8
            bit_offset = bit_pos % 8
            
            if byte_pos + 2 >= len(payload):
                break
                
            # Extract 11 bits across bytes
            if bit_offset <= 5:
                # All bits within 2 bytes
                value = (payload[byte_pos] >> bit_offset) | (payload[byte_pos + 1] << (8 - bit_offset))
            else:
                # Bits span 3 bytes
                value = (payload[byte_pos] >> bit_offset) | (payload[byte_pos + 1] << (8 - bit_offset)) | (payload[byte_pos + 2] << (16 - bit_offset))
            
            # Mask to 11 bits
            channels.append(value & 0x7FF)
        
        return channels
    
    def _crsf_to_rc(self, crsf_value):
        """Convert CRSF value (0-1984) to RC value (1000-2000μs)"""
        return int((crsf_value / 1.984) + 1000)
    
    def _get_control_arrow(self, channel_index, rc_value):
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
    
    def _get_aux_state(self, rc_value):
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
    
    def _print_statistics(self):
        """Print periodic statistics"""
        while self.running:
            time.sleep(5)  # Print stats every 5 seconds
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

# Alternative: Simple receiver that only shows RC data
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
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    if data:
                        self.process_simple(data, addr)
                        
                except socket.timeout:
                    continue
                except KeyboardInterrupt:
                    break
                    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self.sock:
                self.sock.close()
    
    def process_simple(self, data, addr):
        """Simple processing that only extracts RC values"""
        try:
            # Extract frame data (skip header if present)
            if len(data) >= 4:
                frame_size = struct.unpack('>I', data[:4])[0]
                if len(data) == frame_size + 4:
                    frame_data = data[4:]
                else:
                    frame_data = data
            else:
                frame_data = data
            
            # Check if it's an RC channels packet
            if len(frame_data) >= 3 and frame_data[2] == 0x16:
                payload = frame_data[3:-1]  # Skip header and CRC
                if len(payload) >= 22:
                    channels = self._unpack_11bit_channels(payload)
                    
                    # Convert to RC values and display
                    rc_values = [self._crsf_to_rc(ch) for ch in channels[:8]]  # First 8 channels
                    
                    print(f"RC: R:{rc_values[0]:4d} P:{rc_values[1]:4d} T:{rc_values[2]:4d} Y:{rc_values[3]:4d} "
                          f"| AUX: {rc_values[4]:4d} {rc_values[5]:4d} {rc_values[6]:4d} {rc_values[7]:4d}")
        
        except Exception as e:
            pass  # Silent fail for simple receiver
    
    def _unpack_11bit_channels(self, payload):
        """Same unpacking method as above"""
        channels = []
        for i in range(16):
            bit_pos = i * 11
            byte_pos = bit_pos // 8
            bit_offset = bit_pos % 8
            
            if byte_pos + 2 >= len(payload):
                break
                
            if bit_offset <= 5:
                value = (payload[byte_pos] >> bit_offset) | (payload[byte_pos + 1] << (8 - bit_offset))
            else:
                value = (payload[byte_pos] >> bit_offset) | (payload[byte_pos + 1] << (8 - bit_offset)) | (payload[byte_pos + 2] << (16 - bit_offset))
            
            channels.append(value & 0x7FF)
        return channels
    
    def _crsf_to_rc(self, crsf_value):
        """Same conversion method as above"""
        return int((crsf_value / 1.984) + 1000)

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