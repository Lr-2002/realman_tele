#!/usr/bin/env python3
"""
Demo script for testing the Inspire Hand control pipeline.
This demonstrates how to use the Inspire Hand control classes.
"""

import time
import argparse
from inspire_hand import InspireHandModel, InspireHandController, InspireHandUnit

def main():
    parser = argparse.ArgumentParser(description="Demo for Inspire Hand control")
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', 
                        help='Serial port for CAN interface')
    parser.add_argument('--id', type=str, default='01', 
                        help='Device ID (binary format)')
    args = parser.parse_args()

    print(f"Initializing Inspire Hand control on port {args.port} with ID {args.id}")
    
    # Create the model, controller, and unit
    model = InspireHandModel()
    controller = InspireHandController(port=args.port, id_value=args.id)
    hand = InspireHandUnit(controller=controller, model=model, device_name="inspire_hand")
    
    # Initialize the hand
    print("Initializing hand connection...")
    if not hand.initialize():
        print("Failed to initialize hand. Exiting.")
        return

    try:
        print("Hand connection successful!")
        
        # Set finger speeds for smooth movement
        print("Setting finger speeds...")
        hand.set_finger_speeds([300, 300, 300, 300, 300, 300])
        time.sleep(1)
        
        # Demo: Open hand
        print("Opening hand...")
        hand.set_finger_positions([0, 0, 0, 0, 0, 0])
        time.sleep(3)
        
        # Demo: Close hand
        print("Closing hand...")
        hand.set_finger_positions([1000, 1000, 1000, 1000, 1000, 0])
        time.sleep(3)
        
        
    except KeyboardInterrupt:
        print("Demo interrupted")
    except Exception as e:
        print(f"Error during demo: {e}")
    finally:
        # Clean up
        print("Closing connection...")
        hand.close()
        print("Demo completed")

if __name__ == "__main__":
    main()
