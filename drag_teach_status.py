#!/usr/bin/env python3
"""
Drag Teaching Status Monitor for Realman Robotic Arm.
This script demonstrates how to check if the robot arm is in teaching mode.
"""

import time
import signal
import sys
from robotic_arm_package.robotic_arm import Arm

class DragTeachStatus:
    def __init__(self, ip_address="169.254.128.18"):
        """
        Initialize the drag teaching status checker
        
        Args:
            ip_address: IP address of the robot arm
        """
        self.ip_address = ip_address
        self.robot = None
        
        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def connect(self):
        """Connect to the robot arm"""
        print(f"Connecting to robot arm at {self.ip_address}...")
        try:
            # Initialize the arm with device model and IP address
            # Use 0 as device_mode (deprecated parameter)
            self.robot = Arm(0, self.ip_address)
            
            # Check if the socket handle is valid (a positive integer)
            if hasattr(self.robot, 'nSocket') and self.robot.nSocket > 0:
                print(f"Successfully connected to robot arm with socket handle: {self.robot.nSocket}")
                return True
            else:
                print("Failed to connect to robot arm: invalid socket handle")
                return False
        except Exception as e:
            print(f"Error connecting to robot arm: {e}")
            return False
    
    def is_teaching_mode_active(self):
        """
        Check if teaching mode is active by analyzing the arm state
        
        Returns:
            bool: True if teaching mode is active, False otherwise
        """
        if not self.robot:
            print("Robot not connected")
            return False
        
        # Method 1: Check arm all state to see control state
        try:
            result, joint_status = self.robot.Get_Arm_All_State()
            if result == 0:
                # Check the arm state parameters
                # If we have access to the teaching mode state through joint_status
                print(f"Robot state retrieved, checking mode indicators")
                
                # The joint state data structure is complex, but we can look at certain fields
                # that might indicate teaching mode, like error_code or mode
                if hasattr(joint_status, 'mode') and joint_status.mode == 11:  # Mode 11 often indicates teaching
                    return True
                    
                return False  # If no clear indicator found, assume not in teaching mode
            else:
                print(f"Failed to get arm state: error code {result}")
        except Exception as e:
            print(f"Error checking arm state: {e}")
        
        # Method 2: Attempt to start teaching mode
        # If already in teaching mode, this should return a specific error code
        try:
            result = self.robot.Start_Drag_Teach(block=False)
            # If we're already in drag teaching mode, Start_Drag_Teach should return a specific error
            # code (often 40020 or similar) indicating the robot is already in that state
            if result == 40020:  # This code may vary depending on the robot model
                return True
                
            # If the command succeeds, we weren't in teaching mode but now we are
            # So stop teaching mode to return to original state
            if result == 0:
                self.robot.Stop_Drag_Teach(block=True)
                return False
        except Exception as e:
            print(f"Error testing teaching mode state: {e}")
        
        return False
    
    def monitor_teaching_mode(self, interval=1.0):
        """
        Continuously monitor teaching mode status
        
        Args:
            interval: Time between checks in seconds
        """
        try:
            while True:
                is_teaching = self.is_teaching_mode_active()
                print(f"Teaching mode active: {is_teaching}")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped")
    
    def disconnect(self):
        """Disconnect from the robot arm"""
        if not self.robot:
            return
            
        print("Disconnecting from robot arm...")
        self.robot.Socket_Close()
        print("Disconnected")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C to exit gracefully"""
        print("\nExiting...")
        self.disconnect()
        sys.exit(0)

def main():
    """Main function for the drag teaching status monitor"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Drag Teaching Status Monitor for Realman Robot Arm")
    parser.add_argument('--ip', type=str, default="169.254.128.18", 
                        help='IP address of the robot arm')
    args = parser.parse_args()
    
    # Create and run the monitor
    monitor = DragTeachStatus(ip_address=args.ip)
    if monitor.connect():
        try:
            # Check once
            is_teaching = monitor.is_teaching_mode_active()
            print(f"Teaching mode active: {is_teaching}")
            
            # Ask user if they want to continuously monitor
            response = input("Would you like to continuously monitor teaching mode status? (y/n): ")
            if response.lower() == 'y':
                print("Press Ctrl+C to stop monitoring")
                monitor.monitor_teaching_mode()
        finally:
            monitor.disconnect()

if __name__ == "__main__":
    main()
