"""
Data source implementations for the universal controller framework.
This module provides concrete classes for different input data sources.
"""
import numpy as np
import cv2
import mediapipe as mp
from typing import Dict, Any, List, Optional, Tuple, Union
from .base import DataSource


class VisionDataSource(DataSource):
    """
    Base class for vision-based data sources.
    """
    
    def __init__(self, camera_id: int = 0):
        """
        Initialize a vision data source
        
        Args:
            camera_id: Camera ID for capture device
        """
        self.camera_id = camera_id
        self.capture = None
        self.frame = None
        self._initialized = False
        
    def initialize(self) -> bool:
        """
        Initialize the camera capture
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            self.capture = cv2.VideoCapture(self.camera_id)
            if not self.capture.isOpened():
                print(f"Failed to open camera {self.camera_id}")
                return False
            
            # Read a test frame to ensure camera works
            ret, self.frame = self.capture.read()
            if not ret:
                print(f"Failed to read frame from camera {self.camera_id}")
                return False
                
            self._initialized = True
            return True
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
            
    def validate_data(self) -> Tuple[bool, Optional[str]]:
        """
        Validate that the camera data is valid
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        if not self._initialized:
            return False, "Camera not initialized"
            
        if self.capture is None:
            return False, "Camera capture is None"
            
        if not self.capture.isOpened():
            return False, "Camera is not open"
            
        if self.frame is None:
            return False, "No frame available"
            
        if self.frame.size == 0:
            return False, "Empty frame"
            
        return True, None
            
    def close(self) -> None:
        """
        Close the camera capture
        """
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self._initialized = False


class MediaPipeHandSource(VisionDataSource):
    """
    Hand pose estimation using MediaPipe.
    """
    
    def __init__(self, camera_id: int = 0, max_num_hands: int = 1):
        """
        Initialize MediaPipe hand tracker
        
        Args:
            camera_id: Camera ID for capture device
            max_num_hands: Maximum number of hands to detect
        """
        super().__init__(camera_id)
        self.max_num_hands = max_num_hands
        self.mp_hands = mp.solutions.hands
        self.hands = None
        
    def initialize(self) -> bool:
        """
        Initialize camera and MediaPipe hands
        
        Returns:
            True if initialization successful, False otherwise
        """
        if not super().initialize():
            return False
            
        try:
            # Initialize MediaPipe hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=self.max_num_hands,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.5
            )
            return True
        except Exception as e:
            print(f"MediaPipe initialization error: {e}")
            return False
            
    def validate_data(self) -> Tuple[bool, Optional[str]]:
        """
        Validate that the MediaPipe hand data is valid
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        # 首先验证基类的摄像头数据
        is_valid, error_message = super().validate_data()
        if not is_valid:
            return is_valid, error_message
            
        if self.hands is None:
            return False, "MediaPipe hands not initialized"
            
        return True, None
        
    def get_data(self) -> Dict[str, Any]:
        """
        Get hand pose data from MediaPipe
        
        Returns:
            Dictionary with normalized hand landmark data
            
        Raises:
            ValueError: If data is invalid
        """
        # 验证数据有效性
        is_valid, error_message = self.validate_data()
        if not is_valid:
            raise ValueError(f"Invalid MediaPipe data source: {error_message}")
            
        try:
            # Capture frame
            ret, self.frame = self.capture.read()
            if not ret:
                raise ValueError("Failed to read frame from camera")
                
            # Convert to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Extract landmark data
            hand_data = []
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks[:self.max_num_hands]:
                    # Extract hand landmarks
                    landmarks = []
                    for landmark in hand_landmarks.landmark:
                        landmarks.append({
                            "x": landmark.x,
                            "y": landmark.y,
                            "z": landmark.z,
                            "visibility": landmark.visibility if hasattr(landmark, "visibility") else 1.0
                        })
                    
                    hand_data.append({
                        "landmarks": landmarks,
                        "handedness": "unknown"  # MediaPipe can detect handedness but simplified here
                    })
            
            return {
                "hands": hand_data,
                "image": {
                    "width": self.frame.shape[1],
                    "height": self.frame.shape[0],
                    "channels": self.frame.shape[2]
                },
                "timestamp": cv2.getTickCount() / cv2.getTickFrequency()
            }
        except Exception as e:
            print(f"MediaPipe processing error: {e}")
            raise ValueError(f"MediaPipe data processing failed: {str(e)}")
            
    def close(self) -> None:
        """
        Close camera and MediaPipe resources
        """
        if self.hands is not None:
            self.hands.close()
            self.hands = None
            
        super().close()


class RealSenseHandSource(DataSource):
    """
    Hand tracking using Intel RealSense cameras.
    """
    
    def __init__(self, device_id: str = ""):
        """
        Initialize RealSense hand tracking
        
        Args:
            device_id: RealSense device ID (empty for first available)
        """
        self.device_id = device_id
        self.pipeline = None
        self.align = None
        self._initialized = False
        self.last_frames = None
        
    def initialize(self) -> bool:
        """
        Initialize the RealSense camera
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # Import RealSense library
            import pyrealsense2 as rs
            
            # Create pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Use specified device or first available
            if self.device_id:
                config.enable_device(self.device_id)
                
            # Configure streams
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            profile = self.pipeline.start(config)
            
            # Create alignment object
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            
            self._initialized = True
            return True
        except Exception as e:
            print(f"RealSense initialization error: {e}")
            return False
            
    def validate_data(self) -> Tuple[bool, Optional[str]]:
        """
        Validate that the RealSense data is valid
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        if not self._initialized:
            return False, "RealSense not initialized"
            
        if self.pipeline is None:
            return False, "RealSense pipeline is None"
            
        if self.align is None:
            return False, "RealSense align object is None"
            
        return True, None
        
    def get_data(self) -> Dict[str, Any]:
        """
        Get hand pose data from RealSense and associated hand tracking
        
        Returns:
            Dictionary with hand tracking data
            
        Raises:
            ValueError: If data is invalid
        """
        # 验证数据有效性
        is_valid, error_message = self.validate_data()
        if not is_valid:
            raise ValueError(f"Invalid RealSense data source: {error_message}")
            
        try:
            import pyrealsense2 as rs
            import numpy as np
            
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            self.last_frames = frames
            
            # Align depth to color frame
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                raise ValueError("Failed to get valid frames from RealSense camera")
                
            # Convert frames to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # 检查图像是否有效
            if depth_image.size == 0 or color_image.size == 0:
                raise ValueError("Empty image data from RealSense camera")
            
            # TODO: Implement hand tracking using depth and color images
            # This is a simplified stub and should be replaced with actual hand tracking
            # For now, return empty hand data
            
            return {
                "hands": [],
                "depth": {
                    "width": depth_image.shape[1],
                    "height": depth_image.shape[0],
                },
                "color": {
                    "width": color_image.shape[1],
                    "height": color_image.shape[0],
                    "channels": color_image.shape[2]
                },
                "timestamp": frames.get_timestamp()
            }
        except Exception as e:
            print(f"RealSense processing error: {e}")
            raise ValueError(f"RealSense data processing failed: {str(e)}")
            
    def close(self) -> None:
        """
        Close RealSense pipeline
        """
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            self._initialized = False


class GloveDataSource(DataSource):
    """
    Data source for hand tracking using sensor gloves.
    """
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        """
        Initialize glove data source
        
        Args:
            port: Serial port for the glove
            baudrate: Baud rate for serial communication
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._initialized = False
        self.last_data = None
        
    def initialize(self) -> bool:
        """
        Initialize the serial connection to the glove
        
        Returns:
            True if initialization successful, False otherwise
        """
        # Stub implementation - to be implemented later
        print(f"GloveDataSource.initialize() called for port {self.port}")
        self._initialized = True
        return True
            
    def validate_data(self) -> Tuple[bool, Optional[str]]:
        """
        Validate that the glove data is valid
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        if not self._initialized:
            return False, "Glove not initialized"
        
        # 这里添加更多的验证逻辑，例如检查串口连接状态等
        # 由于这是一个存根实现，我们暂时不添加详细的验证
        
        return True, None
            
    def get_data(self) -> Dict[str, Any]:
        """
        Get hand pose data from the glove
        
        Returns:
            Dictionary with hand pose data
            
        Raises:
            ValueError: If data is invalid
        """
        # 验证数据有效性
        is_valid, error_message = self.validate_data()
        if not is_valid:
            raise ValueError(f"Invalid glove data source: {error_message}")
            
        try:
            # Stub implementation - to be implemented later
            # 生成一些模拟数据
            import time
            import random
            
            # 生成随机的手指弯曲度数据
            fingers = [round(random.uniform(0.1, 0.9), 2) for _ in range(5)]
            
            # 生成随机的手腕姿态数据
            wrist = [round(random.uniform(-0.5, 0.5), 2) for _ in range(3)]
            
            data = {
                "fingers": fingers,  # 归一化的手指弯曲度 (0-1)
                "wrist": wrist,    # 手腕姿态 (roll, pitch, yaw)
                "timestamp": time.time()
            }
            
            self.last_data = data
            return data
        except Exception as e:
            print(f"Glove data processing error: {e}")
            raise ValueError(f"Glove data processing failed: {str(e)}")
            
    def close(self) -> None:
        """
        Close the serial connection
        """
        # Stub implementation - to be implemented later
        print("GloveDataSource.close() called")
        self._initialized = False
