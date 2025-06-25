"""
Hand-specific implementations of the universal controller framework.
This module provides concrete classes for controlling robotic hands.
"""
import socket
import json
import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Union
from .base import DeviceUnit, DeviceController, DeviceModel


class HandUnit(DeviceUnit):
    """
    Specialized device unit for robotic hands.
    Extends the DeviceUnit with hand-specific functionality.
    """
    
    def set_finger_positions(self, finger_positions: List[float]) -> bool:
        """
        Set finger joint positions
        
        Args:
            finger_positions: List of finger positions (normalized 0-1 or degrees, based on implementation)
            
        Returns:
            True if control successful, False otherwise
        """
        return self.control({
            "type": "fingers",
            "positions": finger_positions
        })
    
    def set_grasp_type(self, grasp_type: str, grasp_width: float = 1.0) -> bool:
        """
        Set a pre-defined grasp type
        
        Args:
            grasp_type: Type of grasp (e.g., "pinch", "power", "tripod")
            grasp_width: Width of the grasp (0-1, normalized)
            
        Returns:
            True if control successful, False otherwise
        """
        return self.control({
            "type": "grasp",
            "grasp_type": grasp_type,
            "width": grasp_width
        })
    
    def get_finger_positions(self) -> List[float]:
        """
        Get current finger positions
        
        Returns:
            List of finger positions
        """
        state = self.get_state()
        return state.get("finger_positions", [])
    
    def get_tactile_data(self) -> Dict[str, Any]:
        """
        Get tactile sensing data if available
        
        Returns:
            Dictionary with tactile sensing data
        """
        state = self.get_state()
        return state.get("tactile", {})


class SocketHandController(DeviceController):
    """
    Implementation of DeviceController for controlling hands via socket communication.
    """
    
    def __init__(self, ip: str, port: int):
        """
        Initialize a socket-based hand controller
        
        Args:
            ip: IP address of the hand controller
            port: Port number for communication
        """
        self.ip = ip
        self.port = port
        self.socket = None
    
    def initialize(self) -> bool:
        """
        Initialize the socket connection to the hand
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ip, self.port))
            return True
        except Exception as e:
            print(f"Socket connection error: {e}")
            self.socket = None
            return False
    
    def control(self, control_signal: Dict[str, Any]) -> bool:
        """
        Send control signals to the hand via socket
        
        Args:
            control_signal: Control signal dictionary
            
        Returns:
            True if control successful, False otherwise
        """
        # Stub implementation - to be implemented later
        print("SocketHandController.control() called with:", control_signal)
        return True
    
    def get_state(self) -> Dict[str, Any]:
        """
        Get the current state of the hand via socket
        
        Returns:
            Dictionary with the hand state
        """
        # Stub implementation - to be implemented later
        return {
            "finger_positions": [0.0, 0.0, 0.0, 0.0, 0.0],
            "tactile": {"palm": 0.0, "fingers": [0.0, 0.0, 0.0, 0.0, 0.0]}
        }
    
    def close(self) -> None:
        """
        Close the socket connection
        """
        if self.socket is not None:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None


class CANHandController(DeviceController):
    """
    Implementation of DeviceController for controlling hands via CAN bus.
    """
    
    def __init__(self, can_interface: str = "can0"):
        """
        Initialize a CAN bus hand controller
        
        Args:
            can_interface: Name of the CAN interface
        """
        self.can_interface = can_interface
        self.can_bus = None
    
    def initialize(self) -> bool:
        """
        Initialize the CAN connection to the hand
        
        Returns:
            True if connection successful, False otherwise
        """
        # Stub implementation - to be implemented later
        print(f"CANHandController.initialize() called for interface {self.can_interface}")
        return True
    
    def control(self, control_signal: Dict[str, Any]) -> bool:
        """
        Send control signals to the hand via CAN bus
        
        Args:
            control_signal: Control signal dictionary
            
        Returns:
            True if control successful, False otherwise
        """
        # Stub implementation - to be implemented later
        print("CANHandController.control() called with:", control_signal)
        return True
    
    def get_state(self) -> Dict[str, Any]:
        """
        Get the current state of the hand via CAN bus
        
        Returns:
            Dictionary with the hand state
        """
        # Stub implementation - to be implemented later
        return {
            "finger_positions": [0.0, 0.0, 0.0, 0.0, 0.0],
            "tactile": {"palm": 0.0, "fingers": [0.0, 0.0, 0.0, 0.0, 0.0]}
        }
    
    def close(self) -> None:
        """
        Close the CAN connection
        """
        # Stub implementation - to be implemented later
        print("CANHandController.close() called")


class DexterousHandModel(DeviceModel):
    """
    Implementation of DeviceModel for dexterous robotic hands.
    """
    
    def __init__(self, num_fingers: int = 5, degrees_of_freedom: int = 16):
        """
        Initialize a dexterous hand model
        
        Args:
            num_fingers: Number of fingers
            degrees_of_freedom: Total degrees of freedom
        """
        self.num_fingers = num_fingers
        self.degrees_of_freedom = degrees_of_freedom
        
        # Pre-defined grasp types
        self.grasp_configs = {
            "power": [0.9, 0.9, 0.9, 0.9, 0.9],  # Full close for power grasp
            "pinch": [0.9, 0.9, 0.0, 0.0, 0.0],  # Thumb and index only
            "tripod": [0.9, 0.9, 0.9, 0.0, 0.0],  # Thumb, index, middle
            "open": [0.0, 0.0, 0.0, 0.0, 0.0],    # Full open
        }
    
    def convert_to_control_signal(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert standardized control data to hand-specific control signals
        
        Args:
            data: Standardized control data
            
        Returns:
            Hand-specific control signals
        """
        if data["type"] == "fingers":
            # Direct finger position control
            return {
                "command": "set_finger_positions",
                "positions": data["positions"]
            }
        elif data["type"] == "grasp":
            # Pre-defined grasp type
            grasp_type = data["grasp_type"]
            width = data.get("width", 1.0)
            
            if grasp_type in self.grasp_configs:
                # Scale positions by width
                positions = [p * width for p in self.grasp_configs[grasp_type]]
                return {
                    "command": "set_finger_positions",
                    "positions": positions
                }
            else:
                raise ValueError(f"Unsupported grasp type: {grasp_type}")
        else:
            raise ValueError(f"Unsupported control type: {data['type']}")
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the hand model
        
        Returns:
            Dictionary with model information
        """
        return {
            "type": "Dexterous Hand",
            "num_fingers": self.num_fingers,
            "degrees_of_freedom": self.degrees_of_freedom,
            "available_grasps": list(self.grasp_configs.keys())
        }
