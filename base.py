"""
Base classes and interfaces for the universal controller framework.
This module defines the core abstractions for device control.
"""
from abc import ABC, abstractmethod
import json
from typing import Dict, Any, List, Optional, Tuple, Union


class DeviceModel(ABC):
    """
    Abstract interface for device models.
    Device models handle translating standard control data to device-specific formats.
    """
    @abstractmethod
    def validate_data(self, data: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """
        Validate that the input data is correctly formatted for this model
        
        Args:
            data: Standardized control data in dictionary format
            
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if data is valid, False otherwise
            - error_message: None if valid, error message string if invalid
        """
        pass
        
    @abstractmethod
    def convert_to_control_signal(self, data: Dict[str, Any]) -> Any:
        """
        Convert standardized data to device-specific control signals
        
        Args:
            data: Standardized control data in dictionary format
            
        Returns:
            Device-specific control signal format
            
        Raises:
            ValueError: If the data is invalid
        """
        pass
        
    @abstractmethod
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the model
        
        Returns:
            Dictionary containing model information
        """
        pass


class DeviceController(ABC):
    """
    Abstract interface for device controllers.
    Device controllers handle communication with physical devices.
    """
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the controller connection
        
        Returns:
            True if initialization successful, False otherwise
        """
        pass
        
    @abstractmethod
    def validate_control_signal(self, control_signal: Any) -> Tuple[bool, Optional[str]]:
        """
        Validate that the control signal is valid for this controller
        
        Args:
            control_signal: Device-specific control signal
            
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if signal is valid, False otherwise
            - error_message: None if valid, error message string if invalid
        """
        pass
        
    @abstractmethod
    def control(self, control_signal: Any) -> bool:
        """
        Send control signals to the device
        
        Args:
            control_signal: Device-specific control signal
            
        Returns:
            True if control successful, False otherwise
            
        Raises:
            ValueError: If the control signal is invalid
        """
        pass
        
    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """
        Get the current device state
        
        Returns:
            Dictionary containing device state information
        """
        pass
        
    @abstractmethod
    def close(self) -> None:
        """
        Close the controller connection
        """
        pass


class DeviceUnit(ABC):
    """
    Base class for device units.
    A device unit combines a controller and model to provide high-level control.
    """
    def __init__(self, controller: DeviceController, model: DeviceModel, device_name: str):
        """
        Initialize a device unit
        
        Args:
            controller: Device controller for hardware communication
            model: Device model for data transformation
            device_name: Name of the device
        """
        self.controller = controller
        self.model = model
        self.device_name = device_name
        self._initialized = False
        
    def initialize(self) -> bool:
        """
        Initialize the device
        
        Returns:
            True if initialization successful, False otherwise
        """
        result = self.controller.initialize()
        self._initialized = result
        return result
        
    def control(self, data: Dict[str, Any]) -> bool:
        """
        Control the device with standardized data
        
        Args:
            data: Standardized control data
            
        Returns:
            True if control successful, False otherwise
            
        Raises:
            RuntimeError: If the device is not initialized
            ValueError: If the data is invalid
        """
        if not self._initialized:
            raise RuntimeError(f"Device {self.device_name} is not initialized")
            
        # Validate input data
        is_valid, error_message = self.model.validate_data(data)
        if not is_valid:
            raise ValueError(f"Invalid input data for device {self.device_name}: {error_message}")
            
        # Convert standardized data to device-specific control signal
        control_signal = self.model.convert_to_control_signal(data)
        
        # Validate control signal
        is_valid, error_message = self.controller.validate_control_signal(control_signal)
        if not is_valid:
            raise ValueError(f"Invalid control signal for device {self.device_name}: {error_message}")
        
        # Send control signal to the device
        return self.controller.control(control_signal)
        
    def get_state(self) -> Dict[str, Any]:
        """
        Get the current device state
        
        Returns:
            Dictionary containing device state
        """
        if not self._initialized:
            raise RuntimeError(f"Device {self.device_name} is not initialized")
            
        return self.controller.get_state()
        
    def close(self) -> None:
        """
        Close the device connection
        """
        if self._initialized:
            self.controller.close()
            self._initialized = False


class DataSource(ABC):
    """
    Abstract interface for data sources.
    Data sources provide input data for controlling devices.
    """
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the data source
        
        Returns:
            True if initialization successful, False otherwise
        """
        pass
        
    @abstractmethod
    def validate_data(self) -> Tuple[bool, Optional[str]]:
        """
        Validate that the data source is providing valid data
        
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if data is valid, False otherwise
            - error_message: None if valid, error message string if invalid
        """
        pass
        
    @abstractmethod
    def get_data(self) -> Dict[str, Any]:
        """
        Get data from the source
        
        Returns:
            Dictionary containing standardized data
            
        Raises:
            ValueError: If the data is invalid
        """
        pass
        
    @abstractmethod
    def close(self) -> None:
        """
        Close the data source
        """
        pass


class RobotController:
    """
    High-level controller for managing multiple devices and data sources.
    """
    def __init__(self):
        """
        Initialize the robot controller
        """
        self.devices = {}  # Dictionary of device units
        self.data_sources = {}  # Dictionary of data sources
        
    def add_device(self, device_name: str, device: DeviceUnit) -> None:
        """
        Add a device to the controller
        
        Args:
            device_name: Name of the device
            device: Device unit instance
        """
        self.devices[device_name] = device
        
    def add_data_source(self, source_name: str, data_source: DataSource) -> None:
        """
        Add a data source to the controller
        
        Args:
            source_name: Name of the data source
            data_source: Data source instance
        """
        self.data_sources[source_name] = data_source
        
    def initialize(self) -> Dict[str, bool]:
        """
        Initialize all devices and data sources
        
        Returns:
            Dictionary with initialization results for each component
        """
        results = {}
        
        # Initialize data sources
        for name, source in self.data_sources.items():
            results[f"source:{name}"] = source.initialize()
            
        # Initialize devices
        for name, device in self.devices.items():
            results[f"device:{name}"] = device.initialize()
            
        return results
        
    def control(self, mapping: Optional[Dict[str, str]] = None) -> Dict[str, bool]:
        """
        Control devices based on data sources
        
        Args:
            mapping: Dictionary mapping device names to data source names
            
        Returns:
            Dictionary with control results for each device
        """
        if mapping is None:
            mapping = {}
            
        results = {}
        
        # Get data from all sources
        source_data = {}
        for name, source in self.data_sources.items():
            source_data[name] = source.get_data()
            
        # Control devices based on mapping
        for device_name, device in self.devices.items():
            if device_name in mapping:
                source_name = mapping[device_name]
                if source_name in source_data:
                    data = source_data[source_name]
                    results[device_name] = device.control(data)
                else:
                    results[device_name] = False
            
        return results
        
    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        """
        Get states of all devices
        
        Returns:
            Dictionary with states of all devices
        """
        states = {}
        for name, device in self.devices.items():
            try:
                states[name] = device.get_state()
            except Exception as e:
                states[name] = {"error": str(e)}
                
        return states
        
    def close(self) -> None:
        """
        Close all devices and data sources
        """
        # Close data sources
        for source in self.data_sources.values():
            try:
                source.close()
            except Exception:
                pass
                
        # Close devices
        for device in self.devices.values():
            try:
                device.close()
            except Exception:
                pass
