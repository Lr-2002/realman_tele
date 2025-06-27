#!/usr/bin/env python3
"""
RealmanHand: Integrated control for Realman arms and Inspire hands.
Provides unified interface for controlling both left and right arms/hands,
including mirroring functionality.
"""

import time
import threading
import queue
import numpy as np

# Import arm components
from arm import ArmUnit, RealArmController
from realman import RealmanArmModel
from robotic_arm_package.robotic_arm import RM75

# Import hand components
from inspire_hand import InspireHandModel, InspireHandController, InspireHandUnit


class RealmanHand:
    def __init__(
        self,
        left_arm_ip="169.254.128.18",
        right_arm_ip="169.254.128.19",
        left_hand_port="/dev/ttyUSB4",
        right_hand_port="/dev/ttyUSB5",
        left_hand_id="01",
        right_hand_id="01",
    ):
        """
        Initialize connections to the arms and hands.
        
        Args:
            left_arm_ip: IP address of the left arm
            right_arm_ip: IP address of the right arm
            left_hand_port: Serial port for left hand CAN interface
            right_hand_port: Serial port for right hand CAN interface
            left_hand_id: Device ID for left hand (binary format)
            right_hand_id: Device ID for right hand (binary format)
        """
        self.initialized = {
            "left_arm": False,
            "right_arm": False,
            "left_hand": False,
            "right_hand": False
        }
        
        # Message queues for each component
        self.left_arm_queue = queue.Queue(maxsize=1)
        self.right_arm_queue = queue.Queue(maxsize=1)
        self.left_hand_queue = queue.Queue(maxsize=1)
        self.right_hand_queue = queue.Queue(maxsize=1)
        
        # Control flags
        self.running = True
        self.threads = []
        
        # Initialize arms
        print("Initializing arms...")
        self.init_arms(left_arm_ip, right_arm_ip)
        
        # Initialize hands
        print("Initializing hands...")
        self.init_hands(left_hand_port, right_hand_port, left_hand_id, right_hand_id)
        
        print("Initialization complete.")
        print(f"Arm status: Left={self.initialized['left_arm']}, Right={self.initialized['right_arm']}")
        print(f"Hand status: Left={self.initialized['left_hand']}, Right={self.initialized['right_hand']}")
        
        # Start control threads
        self.start_control_threads()
        
        input('Control threads started. Press Enter to continue...') 
        
    def init_arms(self, left_arm_ip, right_arm_ip):
        """Initialize both arms"""
        # Initialize left arm
        try:
            print(f"Connecting to left arm at {left_arm_ip}...")
            left_arm_model = RealmanArmModel()
            left_arm_controller = RealArmController(RM75, left_arm_ip)
            self.left_arm = ArmUnit(left_arm_controller, left_arm_model, device_name="left_arm")
            
            if self.left_arm.initialize():
                print("Left arm initialized successfully")
                self.initialized["left_arm"] = True
            else:
                print("Failed to initialize left arm")
        except Exception as e:
            print(f"Error initializing left arm: {e}")
            
        # Initialize right arm
        try:
            print(f"Connecting to right arm at {right_arm_ip}...")
            right_arm_model = RealmanArmModel()
            right_arm_controller = RealArmController(RM75, right_arm_ip)
            self.right_arm = ArmUnit(right_arm_controller, right_arm_model, device_name="right_arm")
            
            if self.right_arm.initialize():
                print("Right arm initialized successfully")
                self.initialized["right_arm"] = True
            else:
                print("Failed to initialize right arm")
        except Exception as e:
            print(f"Error initializing right arm: {e}")
            
    def init_hands(self, left_hand_port, right_hand_port, left_hand_id, right_hand_id):
        """Initialize both hands"""
        # Initialize left hand
        try:
            print(f"Connecting to left hand on port {left_hand_port} with ID {left_hand_id}...")
            left_hand_model = InspireHandModel()
            left_hand_controller = InspireHandController(port=left_hand_port, id_value=left_hand_id)
            self.left_hand = InspireHandUnit(
                controller=left_hand_controller,
                model=left_hand_model,
                device_name="left_hand"
            )
            
            if self.left_hand.initialize():
                print("Left hand initialized successfully")
                self.initialized["left_hand"] = True
            else:
                print("Failed to initialize left hand")
        except Exception as e:
            print(f"Error initializing left hand: {e}")
            
        # Initialize right hand
        try:
            print(f"Connecting to right hand on port {right_hand_port} with ID {right_hand_id}...")
            right_hand_model = InspireHandModel()
            right_hand_controller = InspireHandController(port=right_hand_port, id_value=right_hand_id)
            self.right_hand = InspireHandUnit(
                controller=right_hand_controller, 
                model=right_hand_model,
                device_name="right_hand"
            )
            
            if self.right_hand.initialize():
                print("Right hand initialized successfully")
                self.initialized["right_hand"] = True
            else:
                print("Failed to initialize right hand")
        except Exception as e:
            print(f"Error initializing right hand: {e}")

    def mirror(self):
        """
        Mirror the right arm and hand based on the left arm and hand.
        Uses the left arm/hand as reference and applies mirroring transformation
        to control the right arm/hand.
        """
        # Mirror arm
        if self.initialized["left_arm"] and self.initialized["right_arm"]:
            try:
                # Get left arm joint positions
                left_arm_positions = self.left_arm.get_joint_positions()
                print(f"Left arm positions: {left_arm_positions}")
                
                # Calculate mirrored positions for right arm
                # The mirroring transformation flips the signs of specific joints
                # to ensure proper mirroring across the sagittal plane
                right_arm_positions = np.array(left_arm_positions) * np.array(
                    [1, -1, -1, -1, -1, -1, -1]
                )
                
                print(f"Mirrored right arm positions: {right_arm_positions}")
                
                # Move right arm to mirrored position
                success = self.right_arm.move_to_joint_position(right_arm_positions)
                return left_arm_positions, right_arm_positions
                print(f"Arm mirroring {'successful' if success else 'failed'}")
            except Exception as e:
                print(f"Error during arm mirroring: {e}")
        else:
            print("Cannot mirror arms - both arms must be initialized")

        # Mirror hand
        if self.initialized["left_hand"] and self.initialized["right_hand"]:
            try:
                # Get left hand finger positions
                left_hand_positions = self.left_hand.get_finger_positions()
                print(f"Left hand positions: {left_hand_positions}")
                
                # Mirror hand positions (this is a simple copy - adjust if needed)
                # Some hands may need different mirroring transformations
                self.right_hand.set_finger_positions(left_hand_positions)
                print("Hand mirroring completed")
            except Exception as e:
                print(f"Error during hand mirroring: {e}")
        else:
            print("Cannot mirror hands - both hands must be initialized")

    def start_control_threads(self):
        """
        Start the continuous control threads for all components
        """
        if self.initialized["left_arm"]:
            left_arm_thread = threading.Thread(
                target=self._left_arm_control_thread,
                daemon=True
            )
            self.threads.append(left_arm_thread)
            left_arm_thread.start()
            print("Left arm control thread started")
            
        if self.initialized["right_arm"]:
            right_arm_thread = threading.Thread(
                target=self._right_arm_control_thread,
                daemon=True
            )
            self.threads.append(right_arm_thread)
            right_arm_thread.start()
            print("Right arm control thread started")
            
        if self.initialized["left_hand"]:
            left_hand_thread = threading.Thread(
                target=self._left_hand_control_thread,
                daemon=True
            )
            self.threads.append(left_hand_thread)
            left_hand_thread.start()
            print("Left hand control thread started")
            
        if self.initialized["right_hand"]:
            right_hand_thread = threading.Thread(
                target=self._right_hand_control_thread,
                daemon=True
            )
            self.threads.append(right_hand_thread)
            right_hand_thread.start()
            print("Right hand control thread started")
    
    def _left_arm_control_thread(self):
        """
        Thread function for controlling left arm
        """
        print("Left arm control thread running")
        while self.running:
            try:
                # Get latest joint positions from queue (non-blocking)
                try:
                    joint_positions = self.left_arm_queue.get(block=False)
                    self.left_arm.move_to_joint_position(joint_positions)
                except queue.Empty:
                    pass  # No new commands, continue
                
                # Small sleep to avoid busy-waiting
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in left arm control thread: {e}")
    
    def _right_arm_control_thread(self):
        """
        Thread function for controlling right arm
        """
        print("Right arm control thread running")
        while self.running:
            try:
                # Get latest joint positions from queue (non-blocking)
                try:
                    joint_positions = self.right_arm_queue.get(block=False)
                    self.right_arm.move_to_joint_position(joint_positions)
                except queue.Empty:
                    pass  # No new commands, continue
                
                # Small sleep to avoid busy-waiting
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in right arm control thread: {e}")
    
    def _left_hand_control_thread(self):
        """
        Thread function for controlling left hand
        """
        print("Left hand control thread running")
        while self.running:
            try:
                # Get latest finger positions from queue (non-blocking)
                try:
                    finger_positions = self.left_hand_queue.get(block=False)
                    self.left_hand.set_finger_positions(finger_positions)
                except queue.Empty:
                    pass  # No new commands, continue
                
                # Small sleep to avoid busy-waiting
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in left hand control thread: {e}")
    
    def _right_hand_control_thread(self):
        """
        Thread function for controlling right hand
        """
        print("Right hand control thread running")
        while self.running:
            try:
                # Get latest finger positions from queue (non-blocking)
                try:
                    finger_positions = self.right_hand_queue.get(block=False)
                    self.right_hand.set_finger_positions(finger_positions)
                except queue.Empty:
                    pass  # No new commands, continue
                
                # Small sleep to avoid busy-waiting
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in right hand control thread: {e}")
    
    def step(self, action):
        """
        Queue actions for both arms and hands, to be executed by control threads.
        
        Args:
            action: List/array of 2 * (7 + 6) = 26 values:
                   [left_arm(7), left_hand(6), right_arm(7), right_hand(6)]
                   
        Returns:
            Success status (boolean)
        """
        if len(action) != 26:
            print(f"Error: Expected 26 values in action, got {len(action)}")
            return False
            
        # Split the action values
        left_arm_action = action[0:7]
        left_hand_action = action[7:13]
        right_arm_action = action[13:20]
        right_hand_action = action[20:26]
        
        # Queue the actions for each control thread
        if self.initialized["left_arm"]:
            # Clear queue and add new command (only keep most recent)
            while not self.left_arm_queue.empty():
                try:
                    self.left_arm_queue.get_nowait()
                except queue.Empty:
                    break
            self.left_arm_queue.put(left_arm_action)
            
        if self.initialized["right_arm"]:
            # Clear queue and add new command (only keep most recent)
            while not self.right_arm_queue.empty():
                try:
                    self.right_arm_queue.get_nowait()
                except queue.Empty:
                    break
            self.right_arm_queue.put(right_arm_action)
            
        if self.initialized["left_hand"]:
            # Clear queue and add new command (only keep most recent)
            while not self.left_hand_queue.empty():
                try:
                    self.left_hand_queue.get_nowait()
                except queue.Empty:
                    break
            self.left_hand_queue.put(left_hand_action)
            
        if self.initialized["right_hand"]:
            # Clear queue and add new command (only keep most recent)
            while not self.right_hand_queue.empty():
                try:
                    self.right_hand_queue.get_nowait()
                except queue.Empty:
                    break
            self.right_hand_queue.put(right_hand_action)
        
        return True
        
    def close(self):
        """Close all connections and stop threads"""
        print("Closing connections...")
        
        # Stop all control threads
        self.running = False
        
        # Wait a bit for threads to finish
        time.sleep(0.1)
        
        # Set hands to open position if initialized

        print("All connections closed")


def main():
    """Demo script for RealmanHand class"""
    # Initialize with default parameters
    system = RealmanHand()
    
    try:
        # Demo 1: Print current positions
        print("\n=== Demo 1: Current Positions ===")
        if system.initialized["left_arm"]:
            left_pos = system.left_arm.get_joint_positions()
            print(f"Left arm positions: {left_pos}")
        
        if system.initialized["right_arm"]:
            right_pos = system.right_arm.get_joint_positions()
            print(f"Right arm positions: {right_pos}")
        
        # Demo 2: Mirror operation
        print("\n=== Demo 2: Mirror Operation ===")
        la, ra = system.mirror()
        time.sleep(3)  # Give time for the movement to complete
        
        # # Demo 3: Execute simple action
        print("\n=== Demo 3: Execute Simple Action ===")
        # # Example action: home positions for arms and open hands
        # # The actual values should be adjusted based on your specific setup
        action = [
            # Left arm (7 values)
            *la, 
            # Left hand (6 values)
            1000, 1000, 1000, 1000, 1000, 1000,
            # Right arm (7 values)
            *ra ,
            # Right hand (6 values)
            1000, 1000, 1000, 1000, 1000, 1000
        ]
        system.step(action)
        time.sleep(1)
        action = [
            # Left arm (7 values)
            *la, 
            # Left hand (6 values)
            0, 0, 0, 0, 300, 0,
            # Right arm (7 values)
            *ra ,
            # Right hand (6 values)
            0, 0, 0, 0, 300, 0
        ]
        system.step(action)
        time.sleep(3)    
    finally:
        # Clean up
        system.close()


if __name__ == "__main__":
    main()
