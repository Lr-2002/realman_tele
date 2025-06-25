"""
Arm-specific implementations of the universal controller framework.
This module provides concrete classes for controlling robotic arms.
"""

import socket
import json
import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Union
from base import DeviceUnit, DeviceController, DeviceModel
from realman import RealmanArmModel


class ArmUnit(DeviceUnit):
    """
    Specialized device unit for robotic arms.
    Extends the DeviceUnit with arm-specific functionality.
    """

    def move_to_joint_position(self, positions: List[float]) -> bool:
        """
        Move the arm to a specific joint position

        Args:
            positions: List of joint angles in degrees

        Returns:
            True if control successful, False otherwise
        """
        return self.control({"type": "joint", "positions": positions})

    def move_to_cartesian_position(
        self, position: List[float], orientation: List[float]
    ) -> bool:
        """
        Move the arm to a specific Cartesian position and orientation

        Args:
            position: [x, y, z] position in meters
            orientation: Orientation (various formats based on implementation)

        Returns:
            True if control successful, False otherwise
        """
        return self.control(
            {"type": "cartesian", "position": position, "orientation": orientation}
        )

    def get_joint_positions(self) -> List[float]:
        """
        Get current joint positions

        Returns:
            List of joint angles in degrees
        """
        state = self.get_state()
        return state.get("joint_positions", [])

    def get_cartesian_position(self) -> Tuple[List[float], List[float]]:
        """
        Get current Cartesian position and orientation

        Returns:
            Tuple of position [x, y, z] and orientation
        """
        state = self.get_state()
        return (
            state.get("position", [0, 0, 0]),
            state.get("orientation", [0, 0, 0, 1]),  # Default quaternion
        )


class SocketArmController(DeviceController):
    """
    Implementation of DeviceController for controlling arms via socket communication.
    """

    def __init__(self, ip: str, port: int):
        """
        Initialize a socket-based arm controller

        Args:
            ip: IP address of the arm controller
            port: Port number for communication
        """
        self.ip = ip
        self.port = port
        self.socket = None

    def initialize(self) -> bool:
        """
        Initialize the socket connection to the arm

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

    def validate_control_signal(
        self, control_signal: Dict[str, Any]
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate that the control signal is valid for this controller

        Args:
            control_signal: Device-specific control signal

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not isinstance(control_signal, dict):
            return (
                False,
                f"Expected dictionary control signal, got {type(control_signal)}",
            )

        if "command" not in control_signal:
            return False, "Missing 'command' field in control signal"

        cmd = control_signal.get("command")

        if cmd == "joint_position":
            if "data" not in control_signal:
                return False, "Missing 'data' field for joint position command"

            # 确保关节位置数据是数值列表
            joint_data = control_signal.get("data")
            if not isinstance(joint_data, (list, tuple, np.ndarray)):
                return False, f"Expected list for joint data, got {type(joint_data)}"

        elif cmd == "cartesian_position":
            if "position" not in control_signal:
                return False, "Missing 'position' field for cartesian position command"

            if "orientation" not in control_signal:
                return (
                    False,
                    "Missing 'orientation' field for cartesian position command",
                )

            # 确保位置和方向数据是列表
            position = control_signal.get("position")
            orientation = control_signal.get("orientation")

            if not isinstance(position, (list, tuple, np.ndarray)):
                return False, f"Expected list for position data, got {type(position)}"

            if not isinstance(orientation, (list, tuple, np.ndarray)):
                return (
                    False,
                    f"Expected list for orientation data, got {type(orientation)}",
                )

        else:
            return False, f"Unsupported command: {cmd}"

        return True, None

    def control(self, control_signal: Dict[str, Any]) -> bool:
        """
        Send control signals to the arm via socket

        Args:
            control_signal: Control signal dictionary

        Returns:
            True if control successful, False otherwise
        """
        if self.socket is None:
            return False

        try:
            # Convert control signal to JSON string
            cmd_str = json.dumps(control_signal)

            # Send command
            self.socket.send(cmd_str.encode())

            # Receive response (simplified implementation)
            response = self.socket.recv(1024).decode()
            response_dict = json.loads(response)

            # Check if command was successful
            return response_dict.get("success", False)
        except Exception as e:
            print(f"Socket control error: {e}")
            return False

    def get_state(self) -> Dict[str, Any]:
        """
        Get the current state of the arm via socket

        Returns:
            Dictionary with the arm state
        """
        if self.socket is None:
            return {"error": "Not connected"}

        try:
            # Send state request
            self.socket.send(b'{"command": "get_state"}')

            # Receive response
            response = self.socket.recv(1024).decode()
            return json.loads(response)
        except Exception as e:
            print(f"Socket state request error: {e}")
            return {"error": str(e)}

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


class RealArmController(DeviceController):
    """
    Implementation of DeviceController for controlling real RobotArm using the existing API.
    Based on the implementation in vision_base.py.
    """

    def __init__(self, arm_type: int, ip: str):
        """
        Initialize a real arm controller

        Args:
            arm_type: Type of the arm (e.g., RM75)
            ip: IP address of the arm
        """
        self.arm_type = arm_type
        self.ip = ip
        self.arm = None

    def initialize(self) -> bool:
        """
        Initialize the real arm connection

        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Import the RoboticArm package
            from robotic_arm_package.robotic_arm import Arm, RM75

            # Create arm instance
            self.arm = Arm(self.arm_type, self.ip)

            # Set run mode (0: simulation, 1: real)
            self.arm.Set_Arm_Run_Mode(1)

            # Initialize to a default position
            # self.arm.Movej_Cmd(np.array([0, 25, 0, 80, 0, 75, 0]), 10, 0, 0, 1)
            tag, joint_list = self.arm.Get_Joint_Degree()
            return True
        except Exception as e:
            print(f"Real arm initialization error: {e}")
            self.arm = None
            return False

    def validate_control_signal(
        self, control_signal: Dict[str, Any]
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate that the control signal is valid for this controller

        Args:
            control_signal: Device-specific control signal

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not isinstance(control_signal, dict):
            return (
                False,
                f"Expected dictionary control signal, got {type(control_signal)}",
            )

        if "command" not in control_signal:
            return False, "Missing 'command' field in control signal"

        cmd = control_signal.get("command")

        if cmd == "joint_position":
            if "data" not in control_signal:
                return False, "Missing 'data' field for joint position command"

            # 验证关节位置数据
            joint_data = control_signal.get("data")
            if not isinstance(joint_data, (list, tuple, np.ndarray)):
                return (
                    False,
                    f"Expected array-like for joint data, got {type(joint_data)}",
                )

            # 验证关节数量是否正确
            if isinstance(joint_data, (list, tuple)) and len(joint_data) != 7:
                return (
                    False,
                    f"Realman arm requires 7 joint values, got {len(joint_data)}",
                )
            elif isinstance(joint_data, np.ndarray) and joint_data.size != 7:
                return (
                    False,
                    f"Realman arm requires 7 joint values, got {joint_data.size}",
                )

        elif cmd == "cartesian_position":
            if "position" not in control_signal:
                return False, "Missing 'position' field for cartesian position command"

            if "orientation" not in control_signal:
                return (
                    False,
                    "Missing 'orientation' field for cartesian position command",
                )

            # 验证位置和方向数据
            position = control_signal.get("position")
            if not isinstance(position, (list, tuple, np.ndarray)):
                return (
                    False,
                    f"Expected array-like for position data, got {type(position)}",
                )

            # 验证位置坐标是否是3维
            if (isinstance(position, (list, tuple)) and len(position) != 3) or (
                isinstance(position, np.ndarray) and position.size != 3
            ):
                return (
                    False,
                    f"Position should have 3 values (x,y,z), got {len(position) if isinstance(position, (list, tuple)) else position.size}",
                )

            # 验证姿态数据
            orientation = control_signal.get("orientation")
            if not isinstance(orientation, (list, tuple, np.ndarray)):
                return (
                    False,
                    f"Expected array-like for orientation data, got {type(orientation)}",
                )

            # 验证姿态是否是3维（欧拉角）
            if (isinstance(orientation, (list, tuple)) and len(orientation) != 3) or (
                isinstance(orientation, np.ndarray) and orientation.size != 3
            ):
                return (
                    False,
                    f"Orientation should have 3 values (euler angles), got {len(orientation) if isinstance(orientation, (list, tuple)) else orientation.size}",
                )

        else:
            return False, f"Unsupported command: {cmd}"

        return True, None

    def control(self, control_signal: Dict[str, Any]) -> bool:
        """
        Send control signals to the real arm

        Args:
            control_signal: Control signal dictionary

        Returns:
            True if control successful, False otherwise
        """
        if self.arm is None:
            return False

        try:
            cmd_type = control_signal.get("command", "")

            if cmd_type == "joint_position":
                # Control joint positions
                joint_angles = np.array(
                    control_signal.get("data", [0, 0, 0, 0, 0, 0, 0])
                )
                self.arm.Movej_Cmd(joint_angles, 10, 0, 0, 1)
                return True

            elif cmd_type == "cartesian_position":
                # Control cartesian position
                position = control_signal.get("position", [0, 0, 0])
                orientation = control_signal.get("orientation", [0, 0, 0, 1])
                pose_target = np.array([*position, *orientation])
                self.arm.Movep_CANFD(pose_target, False)
                return True

            return False
        except Exception as e:
            print(f"Real arm control error: {e}")
            return False

    def get_state(self) -> Dict[str, Any]:
        """
        Get the current state of the real arm

        Returns:
            Dictionary with the arm state
        """
        if self.arm is None:
            return {"error": "Not connected"}

        try:
            # Get current joint positions
            joint_pos = self.arm.Get_Joint_Degree()

            # Get current pose
            pose = self.arm.Get_Current_Tool_Frame()

            return {
                "joint_positions": joint_pos.tolist()
                if isinstance(joint_pos, np.ndarray)
                else joint_pos,
                "position": pose[:3].tolist()
                if isinstance(pose, np.ndarray)
                else pose[:3],
                "orientation": pose[3:].tolist()
                if isinstance(pose, np.ndarray)
                else pose[3:],
            }
        except Exception as e:
            print(f"Real arm state request error: {e}")
            return {"error": str(e)}

    def close(self) -> None:
        """
        Close the arm connection
        """
        if self.arm is not None:
            try:
                # Set to simulation mode
                self.arm.Set_Arm_Run_Mode(0)

                # Close arm connection
                self.arm.close()
            except Exception as e:
                print(f"Real arm close error: {e}")
            self.arm = None


# 注意：UR5ArmModel 已经被 RealmanArmModel 替代，保留此处代码注释仅供参考
# 请使用 from .realman import RealmanArmModel
