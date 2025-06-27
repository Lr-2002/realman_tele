#!/usr/bin/env python3
"""
Teleoperation script that receives UDP control data and maps it to Realman robot arms and hands.
Uses UDPListener to receive data and RealmanHand to control the robot.
"""

import json
import time
import threading
import argparse
import signal
import sys
import numpy as np
from udp_listener import UDPListener
from realman_hand import RealmanHand


class RealmanTeleop:
    def __init__(
        self,
        left_arm_port=9997,
        right_arm_port=9998,
        left_arm_ip="169.254.128.18",
        right_arm_ip="169.254.128.19",
        left_hand_port="/dev/ttyUSB4",
        right_hand_port="/dev/ttyUSB5",
        buffer_size=1024,
    ):
        """
        Initialize teleoperation system
        
        Args:
            left_arm_port: UDP port for left arm control data
            right_arm_port: UDP port for right arm control data
            left_arm_ip: IP address for left robotic arm
            right_arm_ip: IP address for right robotic arm
            left_hand_port: Serial port for left hand
            right_hand_port: Serial port for right hand
            buffer_size: UDP buffer size
        """
        self.left_arm_port = left_arm_port
        self.right_arm_port = right_arm_port
        self.buffer_size = buffer_size
        
        # Initialize robot control system
        print("Initializing RealmanHand system...")
        self.robot = RealmanHand(
            left_arm_ip=left_arm_ip, 
            right_arm_ip=right_arm_ip,
            left_hand_port=left_hand_port,
            right_hand_port=right_hand_port
        )
        
        # Initialize UDP listeners
        self.listener = UDPListener(ports=[left_arm_port, right_arm_port], buffer_size=buffer_size)
        
        # Store latest received data
        self.left_arm_data = None
        self.right_arm_data = None
        self.running = True
        
    def start(self):
        """Start teleoperation system"""
        # Override UDPListener behavior to process received data
        self.original_listen_on_port = self.listener.listen_on_port
        self.listener.listen_on_port = self.listen_on_port
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Start UDP listener
        self.listener.start()
        
    def listen_on_port(self, port):
        """
        Override of UDPListener's listen_on_port method to handle the received data
        """
        sock = self.listener.setup_socket(port)
        self.listener.sockets.append(sock)
        
        while self.listener.running:
            try:
                data, addr = sock.recvfrom(self.buffer_size)
                try:
                    # Try to parse as JSON
                    message = data.decode('utf-8')
                    json_data = json.loads(message)
                    
                    # Process based on port
                    if port == self.left_arm_port:
                        self.left_arm_data = json_data
                        print(f"Received left arm data from {addr[0]}:{addr[1]}")
                    elif port == self.right_arm_port:
                        self.right_arm_data = json_data
                        print(f"Received right arm data from {addr[0]}:{addr[1]}")
                        
                except json.JSONDecodeError:
                    print(f"Invalid JSON received on port {port}")
                except UnicodeDecodeError:
                    print(f"Non-text data received on port {port}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.listener.running:
                    print(f"Error on port {port}: {e}")
                    
        sock.close()
        
    def extract_arm_joint_data(self, data):
        """
        Extract joint data from received JSON
        
        Returns:
            List of 7 joint values for arm
        """
        if not data or 'joint' not in data:
            return None
            
        try:
            # Extract joint values
            # The data format appears to be a list of joint angles
            joint_data = data['joint']
            
            # Ensure we have 7 joint values for the arm
            if isinstance(joint_data, list) and len(joint_data) >= 7:
                return joint_data[:7]  # Take first 7 values for arm
            else:
                print(f"Invalid joint data format: {joint_data}")
                return None
        except Exception as e:
            print(f"Error extracting arm joint data: {e}")
            return None
    
    def extract_hand_data(self, data):
        """
        Extract hand control data from received JSON
        
        Returns:
            List of 6 finger position values
        """
        if not data or 'gripper' not in data:
            return None
            
        try:
            # Extract gripper values
            gripper_data = data['gripper']
            
            # Map gripper data to 6 finger values
            # The specifics of this mapping depend on the actual data format
            if isinstance(gripper_data, list) and len(gripper_data) >= 6:
                return gripper_data[:6]  # Take first 6 values for hand
            elif isinstance(gripper_data, (int, float)):
                # If single value, duplicate it for all fingers
                return [gripper_data] * 6  
            else:
                print(f"Invalid gripper data format: {gripper_data}")
                return None
        except Exception as e:
            print(f"Error extracting hand data: {e}")
            return None
    
    def map_hand_values(self, values):
        """
        Process hand values to ensure they're within valid ranges
        
        Args:
            values: List of finger position values from teleoperation (already in 0-1000 range)
            
        Returns:
            List of 6 mapped finger position values (0-1000 range)
        """
        if not values:
            return [0, 0, 0, 0, 0, 0]
            
        # Values are already in 0-1000 range, just ensure they're within bounds
        try:
            # Clamp values to 0-1000 range and convert to int
            mapped_values = [int(min(max(v, 0), 1000)) for v in values]
            return mapped_values
        except Exception as e:
            print(f"Error processing hand values: {e}")
            return [0, 0, 0, 0, 0, 0]
    
    def control_loop(self):
        """Main control loop that processes data and sends commands to the robot"""
        last_control_time = time.time()
        control_rate = 0.1  # seconds between control commands
        
        while self.running:
            current_time = time.time()
            
            # Limit control rate
            if current_time - last_control_time < control_rate:
                time.sleep(0.01)
                continue
                
            # Extract and process arm joint data
            left_arm_joints = self.extract_arm_joint_data(self.left_arm_data)
            right_arm_joints = self.extract_arm_joint_data(self.right_arm_data)
            
            # Extract and process hand data
            left_hand_data = self.extract_hand_data(self.left_arm_data)
            right_hand_data = self.extract_hand_data(self.right_arm_data)
            
            # Map hand values to appropriate range
            left_hand_positions = self.map_hand_values(left_hand_data)
            right_hand_positions = self.map_hand_values(right_hand_data)
            
            # If we have valid data for all components, send control command
            if left_arm_joints and right_arm_joints and left_hand_positions and right_hand_positions:
                # Combine all data into single action array
                action = [
                    # Left arm (7 values)
                    *left_arm_joints,
                    # Left hand (6 values)
                    *left_hand_positions,
                    # Right arm (7 values)
                    *right_arm_joints,
                    # Right hand (6 values)
                    *right_hand_positions
                ]
                
                # Send control command
                print("Sending control command to robot")
                self.robot.step(action)
                
            # Update control time
            last_control_time = current_time
    
    def stop(self):
        """Stop teleoperation system"""
        print("Stopping teleoperation...")
        self.running = False
        if hasattr(self, 'listener'):
            self.listener.stop()
        if hasattr(self, 'robot'):
            self.robot.close()


def signal_handler(sig, frame):
    """Handle Ctrl+C to clean up"""
    print("\nReceived interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function to run teleoperation"""
    parser = argparse.ArgumentParser(description='Realman Teleoperation')
    parser.add_argument('--left-port', type=int, default=9997,
                        help='UDP port for left arm data (default: 9997)')
    parser.add_argument('--right-port', type=int, default=9998,
                        help='UDP port for right arm data (default: 9998)')
    parser.add_argument('--left-arm-ip', type=str, default='169.254.128.18',
                        help='IP address for left robotic arm')
    parser.add_argument('--right-arm-ip', type=str, default='169.254.128.19',
                        help='IP address for right robotic arm')
    parser.add_argument('--left-hand-port', type=str, default='/dev/ttyUSB4',
                        help='Serial port for left hand')
    parser.add_argument('--right-hand-port', type=str, default='/dev/ttyUSB5',
                        help='Serial port for right hand')
    parser.add_argument('--buffer', type=int, default=1024,
                        help='Size of UDP receive buffer (default: 1024 bytes)')
    
    args = parser.parse_args()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting Realman teleoperation...")
    print("Press Ctrl+C to exit")
    
    teleop = RealmanTeleop(
        left_arm_port=args.left_port,
        right_arm_port=args.right_port,
        left_arm_ip=args.left_arm_ip,
        right_arm_ip=args.right_arm_ip,
        left_hand_port=args.left_hand_port,
        right_hand_port=args.right_hand_port,
        buffer_size=args.buffer
    )
    
    try:
        teleop.start()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()
        print("Teleoperation system stopped")


if __name__ == "__main__":
    main()
