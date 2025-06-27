"""
Inspire Hand control implementation for the universal controller framework.
This module provides classes for controlling Inspire Hand via CAN interface.
"""

import serial
import time
import threading
from typing import Dict, Any, List, Optional, Tuple, Union
from base import DeviceUnit, DeviceController, DeviceModel


# Register dictionary for the Inspire Hand
REGDICT = {
    'ID': 1000,
    'baudrate': 1001,
    'clearErr': 1004,
    'forceClb': 1009,
    'angleSet': 1486,
    'forceSet': 1498,
    'speedSet': 1522,
    'angleAct': 1546,
    'forceAct': 1582,
    'errCode': 1606,
    'statusCode': 1612,
    'temp': 1618,
    'actionSeq': 2320,
    'actionRun': 2322
}


class CANDevice:
    """
    Class for handling CAN communication with the Inspire Hand
    """
    def __init__(self, port: str, id_value: str):
        """
        Initialize a CAN device connection
        
        Args:
            port: Serial port for the CAN interface
            id_value: Device ID in binary string format
        """
        self.port = port
        self.id_value = id_value
        self.ser = self.init_serial()
        
    def init_serial(self):
        """Initialize serial connection"""
        try:
            ser = serial.Serial(self.port, 115200, timeout=1)
            print(f"Serial port {self.port} opened successfully")
            return ser
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            return None
            
    def convert_address_to_binary_write(self, address_dec):
        """Convert decimal address to binary string for write operations"""
        address_bin = bin(address_dec)[2:]
        id_bin = bin(int(self.id_value, 2))[2:].zfill(14)
        formatted_output = f"0000010{address_bin}{id_bin}"  # 6th bit is 1 for write
        return formatted_output
        
    def convert_address_to_binary_read(self, address_dec):
        """Convert decimal address to binary string for read operations"""
        address_bin = bin(address_dec)[2:]
        id_bin = bin(int(self.id_value, 2))[2:].zfill(14)
        formatted_output = f"0000000{address_bin}{id_bin}"  # 6th bit is 0 for read
        return formatted_output
        
    def binary_to_hex(self, binary_string):
        """Convert binary string to hexadecimal"""
        decimal_value = int(binary_string, 2)
        return hex(decimal_value)[2:].upper()
        
    def convert_number_to_bytes(self, number):
        """Convert an integer to a byte array"""
        hex_string = hex(number)[2:].upper()
        if len(hex_string) % 2 != 0:
            hex_string = '0' + hex_string
        byte_array = [int(hex_string[i:i+2], 16) for i in range(0, len(hex_string), 2)][::-1]
        return byte_array
        
    def send_command(self, reg_name: str, values: List[int]):
        """
        Send control command to the hand
        
        Args:
            reg_name: Register name from REGDICT
            values: List of values to send (0-1000, -1 as placeholder)
        """
        if reg_name in ['angleSet', 'forceSet', 'speedSet']:
            val_reg = [val & 0xFFFF for val in values]
            self.write_register(REGDICT[reg_name], val_reg)
        else:
            print('Function call error: str should be one of "angleSet"/"forceSet"/"speedSet", val should be a list of length 6 with values 0-1000, -1 allowed as placeholder')

    def write_register(self, address: int, values: List[int]):
        """
        Write to register and send data
        
        Args:
            address: Register address
            values: Values to write
        """
        # Handle first 4 values
        chunk = values[:4]
        remaining_values = values[4:]

        # Create extended ID for writing
        ext_id_bin = self.convert_address_to_binary_write(address)
        ext_id_hex = self.binary_to_hex(ext_id_bin)
        ext_id_number = int(ext_id_hex, 16)
        ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

        # Build send buffer
        send_buffer = bytearray()
        send_buffer += bytes([0xAA, 0xAA])  # Header
        send_buffer.extend(ext_id_bytes)  # Extended ID

        # Add chunk data
        for value in chunk:
            send_buffer.append(value & 0xFF)  # Low byte
            send_buffer.append(value >> 8)    # High byte

        # Add padding if needed
        data_length = len(chunk) * 2
        if data_length < 8:
            padding_length = 8 - data_length
            send_buffer.extend([0xFF] * padding_length)

        # Add data length and fixed bytes
        send_buffer.append(data_length + (padding_length if data_length < 8 else 0))
        send_buffer.append(0x00)  # Fixed bytes
        send_buffer.append(0x01)
        send_buffer.append(0x00)

        # Calculate checksum
        check_sum = sum(send_buffer[2:]) & 0xFF
        send_buffer.append(check_sum)
        send_buffer += bytes([0x55, 0x55])  # Footer

        # Send the data
        if self.ser:
            self.ser.write(send_buffer)
            print(f"Device {self.id_value} sending command:", send_buffer.hex())
            self.ser.reset_input_buffer()  # Clear input buffer

        # Handle remaining values (if any)
        if remaining_values:
            new_address = address + 8
            
            # Create extended ID for second chunk
            ext_id_bin = self.convert_address_to_binary_write(new_address)
            ext_id_hex = self.binary_to_hex(ext_id_bin)
            ext_id_number = int(ext_id_hex, 16)
            ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

            # Build send buffer for second chunk
            send_buffer = bytearray()
            send_buffer += bytes([0xAA, 0xAA])  # Header
            send_buffer.extend(ext_id_bytes)  # Extended ID

            # Add remaining data
            for value in remaining_values:
                send_buffer.append(value & 0xFF)  # Low byte
                send_buffer.append(value >> 8)    # High byte

            # Add padding if needed
            data_length = len(remaining_values) * 2
            if data_length < 8:
                padding_length = 8 - data_length
                send_buffer.extend([0xFF] * padding_length)

            # Add data length and fixed bytes
            send_buffer.append(0x04)  # Data length (assuming 2 values)
            send_buffer.append(0x00)  # Fixed bytes
            send_buffer.append(0x01)
            send_buffer.append(0x00)

            # Calculate checksum
            check_sum = sum(send_buffer[2:]) & 0xFF
            send_buffer.append(check_sum)
            send_buffer += bytes([0x55, 0x55])  # Footer

            # Send the data
            if self.ser:
                self.ser.write(send_buffer)
                print(f"Device {self.id_value} sending command (part 2):", send_buffer.hex())
                
    def read_register(self, address: int) -> Optional[Tuple[int, ...]]:  
        """
        Read from register and return data
        
        Args:
            address: Register address
            
        Returns:
            Tuple of values read from the register or None if read failed
        """
        if not self.ser:
            print("Serial connection not available")
            return None
            
        # Create extended ID for reading
        ext_id_bin = self.convert_address_to_binary_read(address)
        ext_id_hex = self.binary_to_hex(ext_id_bin)
        ext_id_number = int(ext_id_hex, 16)
        ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

        # Build send buffer
        send_buffer = bytearray()
        send_buffer += bytes([0xAA, 0xAA])  # Header
        send_buffer.extend(ext_id_bytes)  # Extended ID

        # Fixed bytes for read command
        send_buffer.append(0x08)  # Fixed byte
        send_buffer.extend([0x00] * 7)  # Padding
        send_buffer.append(0x01)  # Fixed bytes
        send_buffer.append(0x00)
        send_buffer.append(0x01)    
        send_buffer.append(0x00)

        # Calculate checksum
        check_sum = sum(send_buffer[2:]) & 0xFF
        send_buffer.append(check_sum)
        send_buffer += bytes([0x55, 0x55])  # Footer

        # Send the read command
        self.ser.write(send_buffer)
        print(f"Device {self.id_value} sending read command:", send_buffer.hex())
        self.ser.reset_input_buffer()  # Clear input buffer
        
        # Wait for response
        time.sleep(0.1)  # Wait for device response
        response1 = self.ser.read(23)  # Read response bytes
        
        # Special handling for temperature data
        if address == REGDICT['temp']:
            if len(response1) >= 1:
                frame1_data = response1[6:12]  # Get data bytes
                values1 = [val for val in frame1_data if val <= 60000]  # Filter valid values
                return tuple(values1)
            return None
            
        # For other data types
        values1 = []
        if len(response1) >= 1:
            frame1_data = response1[6:14]  # Get data bytes from first frame
            
            # Convert pairs of bytes to values
            for i in range(0, len(frame1_data), 2):
                if i+1 < len(frame1_data):  # Make sure we have a pair
                    low_byte = frame1_data[i]
                    high_byte = frame1_data[i + 1]
                    value = (high_byte << 8) | low_byte
                    if value > 60000:  # Invalid value
                        value = 0
                    values1.append(value)

            # Read second part of data
            new_address = address + 8
            ext_id_bin = self.convert_address_to_binary_read(new_address)
            ext_id_hex = self.binary_to_hex(ext_id_bin)
            ext_id_number = int(ext_id_hex, 16)
            ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

            # Build send buffer for second request
            send_buffer = bytearray()
            send_buffer += bytes([0xAA, 0xAA])  # Header
            send_buffer.extend(ext_id_bytes)  # Extended ID

            # Fixed bytes for read command
            send_buffer.append(0x04)  # Fixed byte
            send_buffer.extend([0x00] * 7)  # Padding
            send_buffer.append(0x01)  # Fixed bytes
            send_buffer.append(0x00)
            send_buffer.append(0x01)    
            send_buffer.append(0x00)

            # Calculate checksum
            check_sum = sum(send_buffer[2:]) & 0xFF
            send_buffer.append(check_sum)
            send_buffer += bytes([0x55, 0x55])  # Footer

            # Send the command
            self.ser.write(send_buffer)
            self.ser.reset_input_buffer()

            # Wait for response
            time.sleep(0.1)
            response2 = self.ser.read(23)
            print(f"Device {self.id_value} read response (part 2):", response2.hex())

            # Process second response
            values2 = []
            if len(response2) >= 1:
                frame2_data = response2[6:10]  # Data bytes from second frame
                
                # Convert pairs of bytes to values
                for i in range(0, len(frame2_data), 2):
                    if i+1 < len(frame2_data):
                        low_byte = frame2_data[i]
                        high_byte = frame2_data[i + 1]
                        value = (high_byte << 8) | low_byte
                        if value > 60000:  # Invalid value
                            value = 0
                        values2.append(value)

            # Combine values from both responses
            combined_values = values1[:4] + values2[:2]  # First 4 from first frame + 2 from second frame
            print(f"Device {self.id_value} read data:", tuple(combined_values))
            return tuple(combined_values)
            
        return None

    def close(self):
        """Close the serial connection"""
        if self.ser:
            self.ser.close()
            print(f"Serial port {self.port} closed")


class InspireHandModel(DeviceModel):
    """
    Implementation of DeviceModel for Inspire Hand.
    """
    
    def __init__(self):
        """
        Initialize an Inspire Hand model
        """
        self.fingers = 6  # Default number of fingers (5 + palm)
        self.hand_type = "Inspire Hand"
        
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
            
        if data["type"] == "position":
            if "positions" not in data:
                return False, "Missing 'positions' field for position control"
                
            positions = data.get("positions", [])
            if len(positions) != self.fingers:
                return False, f"Expected {self.fingers} finger positions, got {len(positions)}"
                
            # Validate position ranges (0-1000)
            for i, pos in enumerate(positions):
                if not isinstance(pos, (int, float)):
                    return False, f"Finger position {i} is not a number: {pos}"
                
                # Check if position is in valid range or is a placeholder (-1)
                if pos != -1 and (pos < 0 or pos > 1000):
                    return False, f"Finger position {i} out of range (0-1000): {pos}"
        
        elif data["type"] == "force":
            if "forces" not in data:
                return False, "Missing 'forces' field for force control"
                
            forces = data.get("forces", [])
            if len(forces) != self.fingers:
                return False, f"Expected {self.fingers} force values, got {len(forces)}"
                
            # Validate force ranges (0-1000)
            for i, force in enumerate(forces):
                if not isinstance(force, (int, float)):
                    return False, f"Force value {i} is not a number: {force}"
                
                # Check if force is in valid range or is a placeholder (-1)
                if force != -1 and (force < 0 or force > 1000):
                    return False, f"Force value {i} out of range (0-1000): {force}"
        
        elif data["type"] == "speed":
            if "speeds" not in data:
                return False, "Missing 'speeds' field for speed control"
                
            speeds = data.get("speeds", [])
            if len(speeds) != self.fingers:
                return False, f"Expected {self.fingers} speed values, got {len(speeds)}"
                
            # Validate speed ranges (0-1000)
            for i, speed in enumerate(speeds):
                if not isinstance(speed, (int, float)):
                    return False, f"Speed value {i} is not a number: {speed}"
                
                # Check if speed is in valid range or is a placeholder (-1)
                if speed != -1 and (speed < 0 or speed > 1000):
                    return False, f"Speed value {i} out of range (0-1000): {speed}"
        
        else:
            return False, f"Unsupported control type: {data['type']}"
            
        return True, None
        
    def convert_to_control_signal(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert standardized control data to Inspire Hand-specific control signals
        
        Args:
            data: Standardized control data
            
        Returns:
            Inspire Hand-specific control signals
        """
        # First validate the data
        is_valid, error_message = self.validate_data(data)
        if not is_valid:
            raise ValueError(f"Invalid control data: {error_message}")
            
        if data["type"] == "position":
            # Process position data
            return {
                "reg_name": "angleSet",
                "values": data["positions"]
            }
        
        elif data["type"] == "force":
            # Process force data
            return {
                "reg_name": "forceSet",
                "values": data["forces"]
            }
        
        elif data["type"] == "speed":
            # Process speed data
            return {
                "reg_name": "speedSet",
                "values": data["speeds"]
            }
        
        # Should never reach here due to validation
        return {}
        
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the model
        
        Returns:
            Dictionary with model information
        """
        return {
            "name": self.hand_type,
            "type": "robotic_hand",
            "dof": self.fingers,
            "control_modes": ["position", "force", "speed"],
            "position_range": [0, 1000],
            "force_range": [0, 1000],
            "speed_range": [0, 1000]
        }


class InspireHandController(DeviceController):
    """
    Implementation of DeviceController for controlling Inspire Hand using CAN.
    """
    
    def __init__(self, port: str = "/dev/ttyUSB0", id_value: str = "01"):
        """
        Initialize an Inspire Hand controller
        
        Args:
            port: Serial port for CAN interface
            id_value: Device ID in binary format
        """
        self.port = port
        self.id_value = id_value
        self.can_device = None
        
    def initialize(self) -> bool:
        """
        Initialize the CAN connection to the hand
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.can_device = CANDevice(self.port, self.id_value)
            if not self.can_device.ser:
                print(f"Failed to initialize CAN device on port {self.port}")
                return False
                
            # Set default speed for smooth operation
            self.can_device.send_command('speedSet', [1000, 1000, 1000, 1000, 1000, 1000])
            return True
        except Exception as e:
            print(f"Failed to initialize Inspire Hand controller: {e}")
            return False
    
    def validate_control_signal(self, control_signal: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """
        Validate that the control signal is valid for this controller
        
        Args:
            control_signal: Device-specific control signal
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        if "reg_name" not in control_signal:
            return False, "Missing 'reg_name' field in control signal"
            
        reg_name = control_signal.get("reg_name")
        if reg_name not in ["angleSet", "forceSet", "speedSet"]:
            return False, f"Unsupported register name: {reg_name}"
            
        if "values" not in control_signal:
            return False, "Missing 'values' field in control signal"
            
        values = control_signal.get("values", [])
        if len(values) != 6:
            return False, f"Expected 6 values, got {len(values)}"
            
        return True, None
    
    def control(self, control_signal: Dict[str, Any]) -> bool:
        """
        Send control signals to the Inspire Hand via CAN
        
        Args:
            control_signal: Control signal dictionary with 'reg_name' and 'values'
            
        Returns:
            True if control successful, False otherwise
        """
        if not self.can_device:
            print("CAN device not initialized")
            return False
            
        # Validate control signal
        is_valid, error_message = self.validate_control_signal(control_signal)
        if not is_valid:
            print(f"Invalid control signal: {error_message}")
            return False
            
        try:
            # Send command to the hand
            self.can_device.send_command(
                control_signal["reg_name"], 
                control_signal["values"]
            )
            return True
        except Exception as e:
            print(f"Failed to control Inspire Hand: {e}")
            return False
    
    def get_state(self) -> Dict[str, Any]:
        """
        Get the current state of the Inspire Hand
        
        Returns:
            Dictionary with the hand state
        """
        if not self.can_device:
            print("CAN device not initialized")
            return {}
            
        try:
            # Get current finger angles
            angles = self.can_device.read_register(REGDICT["angleAct"])
            
            # Get current finger forces
            forces = self.can_device.read_register(REGDICT["forceAct"])
            
            # Get temperature
            temp = self.can_device.read_register(REGDICT["temp"])
            
            # Get error code
            errors = self.can_device.read_register(REGDICT["errCode"])
            
            return {
                "finger_positions": angles if angles else [0, 0, 0, 0, 0, 0],
                "finger_forces": forces if forces else [0, 0, 0, 0, 0, 0],
                "temperature": temp[0] if temp else 0,
                "error_code": errors[0] if errors else 0
            }
        except Exception as e:
            print(f"Failed to get Inspire Hand state: {e}")
            return {}
    
    def close(self):
        """
        Close the CAN connection
        """
        if self.can_device:
            self.can_device.close()


class InspireHandUnit(DeviceUnit):
    """
    Specialized device unit for Inspire Hand.
    Extends the DeviceUnit with hand-specific functionality.
    """
    
    def __init__(self, controller: DeviceController, model: DeviceModel, device_name: str):
        """
        Initialize an Inspire Hand unit
        
        Args:
            controller: Hand controller for hardware communication
            model: Hand model for data transformation
            device_name: Name of the device
        """
        super().__init__(controller, model, device_name)
    
    def set_finger_positions(self, positions: List[float]) -> bool:
        """
        Set finger positions for the hand
        
        Args:
            positions: List of 6 finger positions (0-1000, -1 for no change)
            
        Returns:
            True if control successful, False otherwise
        """
        return self.control({"type": "position", "positions": positions})
    
    def set_finger_forces(self, forces: List[float]) -> bool:
        """
        Set finger forces for the hand
        
        Args:
            forces: List of 6 finger forces (0-1000, -1 for no change)
            
        Returns:
            True if control successful, False otherwise
        """
        return self.control({"type": "force", "forces": forces})
    
    def set_finger_speeds(self, speeds: List[float]) -> bool:
        """
        Set finger movement speeds for the hand
        
        Args:
            speeds: List of 6 finger speeds (0-1000, -1 for no change)
            
        Returns:
            True if control successful, False otherwise
        """
        return self.control({"type": "speed", "speeds": speeds})
    
    def get_finger_positions(self) -> List[float]:
        """
        Get current finger positions
        
        Returns:
            List of 6 finger positions (0-1000)
        """
        state = self.get_state()
        return state.get("finger_positions", [0, 0, 0, 0, 0, 0])
    
    def get_finger_forces(self) -> List[float]:
        """
        Get current finger forces
        
        Returns:
            List of 6 finger forces (0-1000)
        """
        state = self.get_state()
        return state.get("finger_forces", [0, 0, 0, 0, 0, 0])
    
    def get_temperature(self) -> float:
        """
        Get current hand temperature
        
        Returns:
            Temperature in degrees Celsius
        """
        state = self.get_state()
        return state.get("temperature", 0)
    
    def get_error_code(self) -> int:
        """
        Get current error code
        
        Returns:
            Error code (0 if no error)
        """
        state = self.get_state()
        return state.get("error_code", 0)
