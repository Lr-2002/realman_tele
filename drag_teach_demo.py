#!/usr/bin/env python3
"""
Drag Teaching Mode Demo for Realman Robotic Arm.
This script demonstrates how to put the robot arm into teaching mode
where it can be physically moved by hand.
"""

import time
import argparse
import signal
import sys
from robotic_arm_package.robotic_arm import Arm, RM75

class DragTeachDemo:
    def __init__(self, ip_address="169.254.128.18"):
        """
        Initialize the drag teaching demo
        
        Args:
            ip_address: IP address of the robot arm
        """
        self.ip_address = ip_address
        self.robot = None
        self.is_teaching = False
        
        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def connect(self):
        """Connect to the robot arm"""
        print(f"Connecting to robot arm at {self.ip_address}...")
        try:
            # Initialize the arm with device model and IP address
            # Use 0 as device_mode (deprecated parameter)
            self.robot = Arm(0, self.ip_address)
            
            # The connection is already established in the Arm constructor
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
    
    def start_teaching_mode(self, sensitivity=50):
        """
        Start teaching mode
        
        Args:
            sensitivity: Teaching mode sensitivity (0-100)
        """
        if not self.robot:
            print("Robot not connected")
            return False
            
        print(f"Setting drag teaching sensitivity to {sensitivity}%...")
        self.robot.Set_Drag_Teach_Sensitivity(sensitivity)
        
        print("Starting drag teaching mode...")
        result = self.robot.Start_Drag_Teach(block=True)
        if result != 0:
            print(f"Failed to start teaching mode: error code {result}")
            return False
            
        print("Robot is now in teaching mode")
        print("You can physically move the robot arm")
        self.is_teaching = True
        return True
    
    def stop_teaching_mode(self):
        """Stop teaching mode"""
        if not self.robot or not self.is_teaching:
            return False
            
        print("Stopping teaching mode...")
        result = self.robot.Stop_Drag_Teach(block=True)
        if result != 0:
            print(f"Failed to stop teaching mode: error code {result}")
            return False
            
        print("Robot teaching mode stopped")
        self.is_teaching = False
        return True
    
    def disconnect(self):
        """Disconnect from the robot arm"""
        if not self.robot:
            return
            
        print("Disconnecting from robot arm...")
        self.robot.Socket_Close()
        print("Disconnected")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C to exit gracefully"""
        print("\nExiting demo...")
        if self.is_teaching:
            self.stop_teaching_mode()
        self.disconnect()
        sys.exit(0)
    
    def run(self, sensitivity=50):
        """
        Run the teaching mode demo
        
        Args:
            sensitivity: Teaching mode sensitivity (0-100)
        """
        if not self.connect():
            return
            
        if not self.start_teaching_mode(sensitivity):
            self.disconnect()
            return
            
        print("\n" + "="*50)
        print("DRAG TEACHING MODE ACTIVE")
        print("You can now physically move the robot arm")
        print("Press Ctrl+C to exit teaching mode and quit")
        print("="*50 + "\n")
        
        try:
            # Keep the program running while teaching mode is active
            while self.is_teaching:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting demo...")
        finally:
            if self.is_teaching:
                self.stop_teaching_mode()
            self.disconnect()

def main():
    """Main function for the drag teaching demo"""
    parser = argparse.ArgumentParser(description="Drag Teaching Mode Demo for Realman Robot Arm")
    parser.add_argument('--ip', type=str, default="169.254.128.18", 
                        help='IP address of the robot arm')
    parser.add_argument('--sensitivity', type=int, default=50, 
                        help='Teaching mode sensitivity (0-100)')
    args = parser.parse_args()
    
    # Create and run the demo
    demo = DragTeachDemo(ip_address=args.ip)
    demo.run(sensitivity=args.sensitivity)

if __name__ == "__main__":
    main()
