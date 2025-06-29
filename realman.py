"""
Realman-specific implementations of the universal controller framework.
This module provides concrete classes for controlling Realman robotic arms.
"""
import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Union
from base import DeviceModel


class RealmanArmModel(DeviceModel):
    """
    Implementation of DeviceModel for Realman arm.
    """
    
    def __init__(self, arm_type=None):
        """
        Initialize a Realman arm model
        
        Args:
            arm_type: The specific arm type (RM75, etc.)
        """
        self.arm_type = arm_type
        self.dofs = 7  # 默认自由度
        
    def validate_data(self, data: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """
        Validate that the input data is correctly formatted for this model
        
        Args:
            data: Standardized control data in dictionary format
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        if "type" not in data:
            return False, "Missing 'type' field in control data"
            
        if data["type"] == "joint" or data['type']=='sync':
            if "positions" not in data:
                return False, "Missing 'positions' field for joint control"
                
            positions = data.get("positions", [])
            if len(positions) != self.dofs:
                return False, f"Expected {self.dofs} joint positions, got {len(positions)}"
                
            # Validate joint angle ranges
            for i, pos in enumerate(positions):
                if not isinstance(pos, (int, float)):
                    return False, f"Joint position {i} is not a number: {pos}"
                    
                # 检查关节角度是否在合理范围内 (这里使用一个示例范围，实际应根据具体机械臂型号调整)
                if abs(pos) > 180:
                    return False, f"Joint position {i} out of range: {pos}"
                    
        elif data["type"] == "cartesian":
            if "position" not in data:
                return False, "Missing 'position' field for cartesian control"
                
            if "orientation" not in data:
                return False, "Missing 'orientation' field for cartesian control"
                
            position = data.get("position", [])
            if len(position) != 3:
                return False, f"Expected 3 position values, got {len(position)}"
                
            orientation = data.get("orientation", [])
            if len(orientation) != 3:  # 假设使用欧拉角
                return False, f"Expected 3 orientation values, got {len(orientation)}"
                
            # 检查坐标值是否在工作空间范围内 (这里使用一个示例范围)
            for i, pos in enumerate(position):
                if not isinstance(pos, (int, float)):
                    return False, f"Position component {i} is not a number: {pos}"
                    
                # 检查坐标是否在合理工作空间内
                if abs(pos) > 2.0:  # 假设工作空间限制为±2米
                    return False, f"Position component {i} out of range: {pos}"
                    
        else:
            return False, f"Unsupported control type: {data['type']}"
            
        return True, None
        
    def convert_to_control_signal(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert standardized control data to Realman-specific control signals
        
        Args:
            data: Standardized control data
            
        Returns:
            Realman-specific control signals
        """
        # 首先验证数据
        is_valid, error_message = self.validate_data(data)
        if not is_valid:
            raise ValueError(f"Invalid control data: {error_message}")
            
        if data["type"] == "joint":
            # Process joint position data
            return {
                "command": "joint_position",
                "data": data["positions"]
            }
        elif data['type'] == 'sync':
            return {
                "command": "sync",
                "data": data["positions"]
            }
        elif data["type"] == "cartesian":
            # Process cartesian position data
            return {
                "command": "cartesian_position",
                "position": data["position"],
                "orientation": data["orientation"]
            }
        else:
            raise ValueError(f"Unsupported control type: {data['type']}")
        
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the Realman model
        
        Returns:
            Dictionary with model information
        """
        return {
            "name": "Realman Arm",
            "manufacturer": "睿尔曼机器人(Realman)",
            "arm_type": self.arm_type,
            "dofs": self.dofs,
            "max_reach": 0.85,  # 米，根据实际型号调整
            "max_payload": 5.0   # 公斤，根据实际型号调整
        }
