#!/usr/bin/env python3
"""
AIDA Data Listener
Reads data from AIDA Atom controllers via serial ports and provides access to joint angles.
"""

import serial
import binascii
import time
import threading
import configparser
import json
import socket
import argparse
import signal
import sys
import numpy as np
from datetime import datetime


class DataFilter:
    """Simple data filter to smooth joint values"""
    def __init__(self, depth=10):
        self.depth = depth
        self.filter_array = {}
        
    def Refilter(self, index, value, filter_depth=None):
        """Filter data to reduce noise"""
        if filter_depth is None:
            filter_depth = self.depth
            
        if index not in self.filter_array:
            self.filter_array[index] = [value] * filter_depth
            
        self.filter_array[index].append(value)
        if len(self.filter_array[index]) > filter_depth:
            self.filter_array[index].pop(0)
            
        return np.mean(self.filter_array[index])


class AIDAListener:
    def __init__(
        self, 
        left_port="/dev/rmUSB0", 
        right_port="/dev/rmUSB1", 
        baudrate=460800,
        udp_broadcast=True,
        udp_port_left=9997,
        udp_port_right=9998,
        filter_depth=10
    ):
        """
        Initialize AIDA listener for both left and right controllers
        
        Args:
            left_port: Serial port for left controller
            right_port: Serial port for right controller
            baudrate: Serial baudrate (default: 460800)
            udp_broadcast: Whether to broadcast data over UDP
            udp_port_left: UDP port for left data broadcast
            udp_port_right: UDP port for right data broadcast
            filter_depth: Depth for data filtering
        """
        self.left_port = left_port
        self.right_port = right_port
        self.baudrate = baudrate
        self.udp_broadcast = udp_broadcast
        self.udp_port_left = udp_port_left
        self.udp_port_right = udp_port_right
        
        # Initialize data filters
        self.left_filter = DataFilter(filter_depth)
        self.right_filter = DataFilter(filter_depth)
        
        # Initialize data storage
        self.left_joints = []
        self.right_joints = []
        self.left_grip = 0
        self.right_grip = 0
        
        # Initialize locks for thread safety
        self.data_lock = threading.Lock()
        
        # Running flag
        self.running = True
        
        # Configuration command
        self.config_cmd = "55 AA 02 00 00 67"
        
        # Socket for UDP broadcasting if enabled
        if self.udp_broadcast:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def setup_serial(self):
        """Set up serial connections to controllers"""
        try:
            # Open serial ports
            self.left_ser = serial.Serial(self.left_port, self.baudrate, timeout=0)
            print(f"Connected to left controller on {self.left_port}")
        except Exception as e:
            print(f"Error connecting to left controller: {e}")
            self.left_ser = None
            
        try:
            self.right_ser = serial.Serial(self.right_port, self.baudrate, timeout=0)
            print(f"Connected to right controller on {self.right_port}")
        except Exception as e:
            print(f"Error connecting to right controller: {e}")
            self.right_ser = None
        # Configure controllers
        bytes_to_send = binascii.unhexlify(self.config_cmd.replace(" ", ""))
        
        if self.left_ser:
            self.left_ser.write(bytes_to_send)
        if self.right_ser:
            self.right_ser.write(bytes_to_send)
            
        time.sleep(1)  # Allow time for device to configure
        return self.left_ser is not None or self.right_ser is not None
        
    def bytes_to_signed_int(self, byte_data):
        """Convert bytes to signed integer"""
        value = int.from_bytes(byte_data, byteorder='little', signed=True)
        return value
        
    def parse_joint_data(self, hex_received):
        """
        Parse joint angles from received hex data
        
        Returns:
            tuple: (joints, grip) - list of joint angles and grip value
        """
        if len(hex_received) < 94:
            return [], 0
            
        try:
            # Parse joint angles from hex received data
            J1 = hex_received[14:22]
            J1_byte_data = bytearray.fromhex(J1)
            Joint1 = self.bytes_to_signed_int(J1_byte_data) / 10000.0

            J2 = hex_received[24:32]
            J2_byte_data = bytearray.fromhex(J2)
            Joint2 = self.bytes_to_signed_int(J2_byte_data) / 10000.0

            J3 = hex_received[34:42]
            J3_byte_data = bytearray.fromhex(J3)
            Joint3 = self.bytes_to_signed_int(J3_byte_data) / 10000.0

            J4 = hex_received[44:52]
            J4_byte_data = bytearray.fromhex(J4)
            Joint4 = self.bytes_to_signed_int(J4_byte_data) / 10000.0

            J5 = hex_received[54:62]
            J5_byte_data = bytearray.fromhex(J5)
            Joint5 = self.bytes_to_signed_int(J5_byte_data) / 10000.0

            J6 = hex_received[64:72]
            J6_byte_data = bytearray.fromhex(J6)
            Joint6 = self.bytes_to_signed_int(J6_byte_data) / 10000.0

            if len(hex_received) >= 84:  # Check if we have J7 data
                J7 = hex_received[74:82]
                J7_byte_data = bytearray.fromhex(J7)
                Joint7 = self.bytes_to_signed_int(J7_byte_data) / 10000.0
                Joints = [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, Joint7]
            else:
                Joints = [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
                
            # Get grip data if available
            if len(hex_received) >= 94:
                grip_data = hex_received[84:92]
                grip_byte_data = bytearray.fromhex(grip_data)
                Grip = self.bytes_to_signed_int(grip_byte_data) 
            else:
                Grip = 0
                
            return Joints, Grip
            
        except Exception as e:
            print(f"Error parsing joint data: {e}")
            return [], 0
    
    def filter_joints(self, joints, data_filter):
        """Apply filtering to joint values"""
        if not joints:
            return []
            
        joint_filter = []
        for i in range(len(joints)):
            filtered_value = data_filter.Refilter(i, joints[i], 10)
            joint_filter.append(filtered_value)
            
        return joint_filter
    
    def read_controllers(self):
        """Read data from controllers and update joint values"""
        # Send command to both controllers
        bytes_to_send = binascii.unhexlify(self.config_cmd.replace(" ", ""))
        
        # Read from left controller
        if self.left_ser:
            try:
                self.left_ser.write(bytes_to_send)
                left_bytes_received = self.left_ser.read(self.left_ser.inWaiting())
                print('left byte', left_bytes_received)
                left_hex_received = binascii.hexlify(left_bytes_received).decode('utf-8').upper()
                left_joints, left_grip = self.parse_joint_data(left_hex_received)
                
                if left_joints:
                    # Apply filtering
                    left_joints_filtered = self.filter_joints(left_joints, self.left_filter)
                    
                    # Update stored values with thread safety
                    with self.data_lock:
                        self.left_joints = left_joints_filtered
                        self.left_grip = left_grip
                        
                    # Broadcast over UDP if enabled
                    if self.udp_broadcast:
                        self.broadcast_data(self.udp_port_left, left_joints_filtered, left_grip)
            except Exception as e:
                print(f"Error reading from left controller: {e}")
        
        # Read from right controller
        if self.right_ser:
            try:
                self.right_ser.write(bytes_to_send)
                right_bytes_received = self.right_ser.read(self.right_ser.inWaiting())
                right_hex_received = binascii.hexlify(right_bytes_received).decode('utf-8').upper()
                right_joints, right_grip = self.parse_joint_data(right_hex_received)
                
                if right_joints:
                    # Apply filtering
                    right_joints_filtered = self.filter_joints(right_joints, self.right_filter)
                    
                    # Update stored values with thread safety
                    with self.data_lock:
                        self.right_joints = right_joints_filtered
                        self.right_grip = right_grip
                        
                    # Broadcast over UDP if enabled
                    if self.udp_broadcast:
                        self.broadcast_data(self.udp_port_right, right_joints_filtered, right_grip)
            except Exception as e:
                print(f"Error reading from right controller: {e}")
    
        print(self.left_joints, self.left_grip, self.right_joints, self.right_grip)

    def broadcast_data(self, port, joints, grip):
        """Broadcast joint data over UDP"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # Create JSON data with joint angles and grip
        data = {
            "timestamp": timestamp,
            "joint": joints,
            "gripper": grip
        }
        
        try:
            # Convert to JSON string
            json_data = json.dumps(data)
            
            # Send over UDP
            self.sock.sendto(json_data.encode('utf-8'), ('127.0.0.1', port))
            print(f"[{timestamp}] Broadcasted to port {port}: {json_data}")
        except Exception as e:
            print(f"Error broadcasting data: {e}")
    
    def get_joint_data(self):
        """
        Get the current joint data (thread-safe)
        
        Returns:
            dict: Dictionary with left and right joint data
        """
        with self.data_lock:
            return {
                "left": {
                    "joints": self.left_joints.copy() if self.left_joints else [],
                    "grip": self.left_grip
                },
                "right": {
                    "joints": self.right_joints.copy() if self.right_joints else [],
                    "grip": self.right_grip
                }
            }
    
    def start(self):
        """Start the AIDA listener"""
        if not self.setup_serial():
            print("Failed to setup serial connections. Exiting.")
            return False
        
        # Start reading thread
        self.read_thread = threading.Thread(target=self._read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        return True
    
    def _read_loop(self):
        """Internal loop for reading data"""
        while self.running:
            self.read_controllers()
            time.sleep(0.02)  # 50Hz update rate
    
    def stop(self):
        """Stop the AIDA listener"""
        self.running = False
        time.sleep(0.1)  # Give time for threads to clean up
        
        # Close serial ports
        if hasattr(self, 'left_ser') and self.left_ser:
            self.left_ser.close()
        if hasattr(self, 'right_ser') and self.right_ser:
            self.right_ser.close()
            
        print("AIDA listener stopped")


def signal_handler(sig, frame):
    """Handle interrupt signal"""
    print("\nShutting down AIDA listener...")
    sys.exit(0)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='AIDA Data Listener')
    parser.add_argument('--left-port', type=str, default='/dev/ttyUSB1', 
                        help='Serial port for left controller')
    parser.add_argument('--right-port', type=str, default='/dev/ttyUSB2',
                        help='Serial port for right controller')
    parser.add_argument('--baudrate', type=int, default=460800,
                        help='Serial baudrate')
    parser.add_argument('--left-udp', type=int, default=9997,
                        help='UDP port for left data broadcast')
    parser.add_argument('--right-udp', type=int, default=9998,
                        help='UDP port for right data broadcast')
    parser.add_argument('--no-broadcast', action='store_true',
                        help='Disable UDP broadcasting')
    args = parser.parse_args()
    
    print("Starting AIDA listener...")
    print(f"Left controller: {args.left_port}")
    print(f"Right controller: {args.right_port}")
    
    if not args.no_broadcast:
        print(f"Broadcasting on UDP ports: {args.left_udp} (left), {args.right_udp} (right)")
    
    # Set up signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    listener = AIDAListener(
        left_port=args.left_port,
        right_port=args.right_port,
        baudrate=args.baudrate,
        udp_broadcast=not args.no_broadcast,
        udp_port_left=args.left_udp,
        udp_port_right=args.right_udp
    )
    
    if listener.start():
        # Keep main thread running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()


if __name__ == "__main__":
    main()
