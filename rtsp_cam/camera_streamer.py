#!/usr/bin/env python3
"""
Generic RTSP Camera Streamer Module

This module provides RTSP streaming functionality using GStreamer for optimal performance.
It supports both main and secondary streams with H.264/H.265 encoding and multiple decoder options.
"""

import cv2
import numpy as np
import gi
import threading
import time
from typing import Optional, Callable

# GStreamer imports
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RTSPCameraStreamer:
    """
    Generic RTSP Camera Streamer class for handling RTSP streams using GStreamer.
    
    This class provides efficient streaming capabilities for any RTSP camera
    using GStreamer pipeline with multiple decoder options (software, NVIDIA, Jetson).
    """
    
    def __init__(self, rtsp_url: str, stream_type: str = "main", buffer_size: int = 1, gst_pipeline: str = None, decoder_type: str = "software"):
        """
        Initialize the camera streamer.
        
        Args:
            rtsp_url: RTSP URL for the camera stream
            stream_type: Type of stream ("main" or "sub")
            buffer_size: Buffer size for GStreamer pipeline
            gst_pipeline: Custom GStreamer pipeline string (optional)
            decoder_type: Decoder type ("software", "nvidia", "jetson", "vaapi")
        """
        self.rtsp_url = rtsp_url
        self.stream_type = stream_type
        self.buffer_size = buffer_size
        self.gst_pipeline_str = gst_pipeline
        self.decoder_type = decoder_type
        self.is_running = False
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.bridge = CvBridge()
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create GStreamer pipeline
        self.pipeline = None
        self.appsink = None
        self._create_pipeline()
        
        # Callback for new frames
        self.frame_callback = None
        
    def _create_pipeline(self):
        """Create and configure the GStreamer pipeline."""
        # Use custom pipeline if provided, otherwise create default
        if self.gst_pipeline_str:
            pipeline_str = self.gst_pipeline_str
        else:
            pipeline_str = self._build_pipeline()
        
        # Create pipeline
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        
        # Configure appsink
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("max-buffers", self.buffer_size)
        self.appsink.set_property("drop", True)
        
        # Connect signal
        self.appsink.connect("new-sample", self._on_new_sample, None)
            
    def _build_pipeline(self) -> str:
        """Build GStreamer pipeline based on decoder type and encoding."""
        # Determine decoder elements based on type
        if self.decoder_type == "nvidia":
            h264_decoder = "nvh264dec"
            h265_decoder = "nvh265dec"
        elif self.decoder_type == "jetson":
            h264_decoder = "nvv4l2decoder"
            h265_decoder = "nvv4l2decoder"
        elif self.decoder_type == "vaapi":
            h264_decoder = "vaapih264dec"
            h265_decoder = "vaapih265dec"
        else:  # software
            h264_decoder = "avdec_h264"
            h265_decoder = "avdec_h265"
        
        # Create pipeline string based on encoding
        if "h265" in self.rtsp_url.lower():
            pipeline_str = (
                f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true ! "
                f"rtph265depay ! h265parse ! {h265_decoder} ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink name=appsink"
            )
        else:  # Default to H.264
            pipeline_str = (
                f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true ! "
                f"rtph264depay ! h264parse ! {h264_decoder} ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink name=appsink"
            )
        
        return pipeline_str
        
    def _on_new_sample(self, appsink, data):
        """Callback for new sample from GStreamer pipeline."""
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
            
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        
        # Get video frame dimensions and format
        structure = caps.get_structure(0)
        height = structure.get_value("height")
        width = structure.get_value("width")
        format_str = structure.get_value("format")
        
        # Map buffer to read data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        
        try:
            # Calculate expected buffer size
            if format_str in ['RGB', 'BGR']:
                channels = 3
                expected_size = width * height * channels
            elif format_str in ['RGBA', 'BGRA']:
                channels = 4
                expected_size = width * height * channels
            elif format_str in ['GRAY8', 'GRAY16_LE', 'GRAY16_BE']:
                channels = 1
                expected_size = width * height * channels
            else:
                # Default to BGR
                channels = 3
                expected_size = width * height * channels
            
            # Check if buffer size matches expected size
            actual_size = map_info.size
            if actual_size < expected_size:
                print(f"Warning: Buffer size mismatch. Expected: {expected_size}, Actual: {actual_size}")
                buffer.unmap(map_info)
                return Gst.FlowReturn.OK
            
            # Create numpy array from buffer data
            buffer_array = np.frombuffer(map_info.data, dtype=np.uint8)
            
            # Reshape to image dimensions
            if channels == 3:
                frame = buffer_array[:expected_size].reshape((height, width, 3))
            elif channels == 4:
                frame = buffer_array[:expected_size].reshape((height, width, 4))
                # Convert RGBA to RGB if needed
                frame = frame[:, :, :3]
            else:
                frame = buffer_array[:expected_size].reshape((height, width))
                # Convert grayscale to BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # Copy frame to avoid buffer issues
            frame_copy = frame.copy()
            
            # Update current frame
            with self.frame_lock:
                self.current_frame = frame_copy
            
            # Call frame callback if set
            if self.frame_callback:
                self.frame_callback(frame_copy)
                
        except Exception as e:
            print(f"Error processing frame: {str(e)}")
        finally:
            buffer.unmap(map_info)
            
        return Gst.FlowReturn.OK
    
    def set_frame_callback(self, callback: Callable[[np.ndarray], None]):
        """
        Set callback function for new frames.
        
        Args:
            callback: Function to call when new frame is available
        """
        self.frame_callback = callback
    
    def start(self):
        """Start the camera stream."""
        if not self.is_running:
            self.is_running = True
            
            # Set pipeline state to playing
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f"Failed to start {self.stream_type} stream pipeline")
            
            # Wait for pipeline to be ready
            ret = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
            if ret[0] != Gst.StateChangeReturn.SUCCESS:
                raise RuntimeError(f"Failed to start {self.stream_type} stream")
            
            print(f"Started {self.stream_type} stream from {self.rtsp_url}")
    
    def stop(self):
        """Stop the camera stream."""
        if self.is_running:
            self.is_running = False
            
            # Set pipeline state to null
            self.pipeline.set_state(Gst.State.NULL)
            print(f"Stopped {self.stream_type} stream")
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the current frame.
        
        Returns:
            Current frame as numpy array or None if no frame available
        """
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
        return None
    
    def is_connected(self) -> bool:
        """
        Check if the camera is connected and streaming.
        
        Returns:
            True if connected and streaming, False otherwise
        """
        if not self.is_running:
            return False
        
        state = self.pipeline.get_state(0)[1]
        return state == Gst.State.PLAYING
    
    def __del__(self):
        """Cleanup when object is destroyed."""
        self.stop()


class RTSPCameraNode(Node):
    """
    ROS 2 Node for generic RTSP Camera streaming.
    
    This node publishes camera images and camera info to ROS topics.
    """
    
    def __init__(self, stream_type: str = "main"):
        """
        Initialize the camera node.
        
        Args:
            stream_type: Type of stream ("main" or "sub")
        """
        super().__init__(f'rtsp_cam_{stream_type}_node')
        
        self.stream_type = stream_type
        self.camera_streamer = None
        self.bridge = CvBridge()
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize publishers
        self._init_publishers()
        
        # Initialize camera streamer
        self._init_camera_streamer()
        
        # Create timer for publishing using configured frame rate
        frame_rate = self.get_parameter('frame_rate').value
        self.timer = self.create_timer(1.0/frame_rate, self._publish_frame)
        
        self.get_logger().info(f"RTSP Camera {stream_type} node initialized with {frame_rate} FPS")
    
    def _declare_parameters(self):
        """Declare ROS parameters."""
        # Camera connection parameters
        self.declare_parameter('camera_ip', '192.168.0.18')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')
        self.declare_parameter('camera_port', 554)
        
        # Stream URLs
        self.declare_parameter('main_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/main/av_stream')
        self.declare_parameter('sub_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/sub/av_stream')
        
        # Stream configuration
        self.declare_parameter('main_stream_enabled', True)
        self.declare_parameter('sub_stream_enabled', True)
        self.declare_parameter('encoding', 'h264')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('resolution_width', 1920)
        self.declare_parameter('resolution_height', 1080)
        
        # Camera info parameters
        self.declare_parameter('camera_name', 'rtsp_camera')
        self.declare_parameter('camera_frame_id', 'camera_link')
        
        # Decoder parameters
        self.declare_parameter('decoder_type', 'software')
        
        # Performance parameters
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('timeout_ms', 5000)
        
        # GStreamer pipeline parameters
        self.declare_parameter('gst_pipeline_main', '')
        self.declare_parameter('gst_pipeline_sub', '')
        
    def _init_publishers(self):
        """Initialize ROS publishers."""
        queue_size = self.get_parameter('queue_size').value
        
        # Image publisher
        self.image_pub = self.create_publisher(
            Image,
            'image_raw',
            queue_size
        )
        
        # Camera info publisher
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            'camera_info',
            queue_size
        )
        
    def _init_camera_streamer(self):
        """Initialize the camera streamer."""
        # Get stream URL based on stream type
        if self.stream_type == "main":
            rtsp_url = self.get_parameter('main_stream_url').value
            stream_enabled = self.get_parameter('main_stream_enabled').value
        else:
            rtsp_url = self.get_parameter('sub_stream_url').value
            stream_enabled = self.get_parameter('sub_stream_enabled').value
        
        # Check if stream is enabled
        if not stream_enabled:
            self.get_logger().warn(f"{self.stream_type} stream is disabled in configuration")
            return
        
        # Get performance parameters
        buffer_size = self.get_parameter('buffer_size').value
        decoder_type = self.get_parameter('decoder_type').value
        
        # Get GStreamer pipeline configuration
        if self.stream_type == "main":
            gst_pipeline = self.get_parameter('gst_pipeline_main').value
        else:
            gst_pipeline = self.get_parameter('gst_pipeline_sub').value
        
        # Use custom pipeline if specified, otherwise use empty string (will use default)
        gst_pipeline = gst_pipeline if gst_pipeline else None
        
        # Create camera streamer
        self.camera_streamer = RTSPCameraStreamer(rtsp_url, self.stream_type, buffer_size, gst_pipeline, decoder_type)
        
        # Set frame callback
        self.camera_streamer.set_frame_callback(self._on_new_frame)
        
        # Start streaming
        try:
            self.camera_streamer.start()
            self.get_logger().info(f"Started {self.stream_type} stream")
        except Exception as e:
            self.get_logger().error(f"Failed to start {self.stream_type} stream: {str(e)}")
    
    def _on_new_frame(self, frame):
        """Callback for new frame from camera streamer."""
        # Store frame for publishing
        self.current_frame = frame
    
    def _publish_frame(self):
        """Publish current frame to ROS topic."""
        if hasattr(self, 'current_frame') and self.current_frame is not None:
            try:
                # Convert frame to ROS message
                ros_image = self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = self.get_parameter('camera_frame_id').value
                
                # Publish image
                self.image_pub.publish(ros_image)
                
                # Publish camera info
                self._publish_camera_info()
                
            except Exception as e:
                self.get_logger().error(f"Error publishing frame: {str(e)}")
    
    def _publish_camera_info(self):
        """Publish camera info message."""
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = self.get_parameter('camera_frame_id').value
        
        # Set camera parameters from configuration
        camera_info.width = self.get_parameter('resolution_width').value
        camera_info.height = self.get_parameter('resolution_height').value
        
        # Set distortion model (you may need to calibrate your camera)
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set camera matrix (example values - you should calibrate your camera)
        camera_info.k = [
            camera_info.width * 0.8, 0.0, camera_info.width / 2.0,
            0.0, camera_info.height * 0.8, camera_info.height / 2.0,
            0.0, 0.0, 1.0
        ]
        
        # Set rectification matrix (identity for monocular camera)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Set projection matrix
        camera_info.p = [
            camera_info.k[0], 0.0, camera_info.k[2], 0.0,
            0.0, camera_info.k[4], camera_info.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Publish camera info
        self.camera_info_pub.publish(camera_info)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if self.camera_streamer:
            self.camera_streamer.stop()
        super().destroy_node()


def main(args=None):
    """Main function for the camera node."""
    rclpy.init(args=args)
    
    try:
        # Create camera node (default to main stream)
        camera_node = RTSPCameraNode("main")
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(camera_node)
        
        # Spin the node
        executor.spin()
        
    except KeyboardInterrupt:
        print("Camera node interrupted by user")
    except Exception as e:
        print(f"Error in camera node: {str(e)}")
    finally:
        # Cleanup
        if 'camera_node' in locals():
            camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 