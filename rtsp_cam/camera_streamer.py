#!/usr/bin/env python3
"""
Generic RTSP Camera Streamer Module

This module provides RTSP streaming functionality using GStreamer for optimal performance.
It supports both main and secondary streams with H.264/H.265 encoding and multiple decoder options.
"""

# OpenCV not needed for pure GStreamer streaming
import numpy as np
import gi
import threading
import time
from typing import Optional, Callable

# GStreamer imports
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

# ROS 1 imports
import rospy
from sensor_msgs.msg import Image, CameraInfo


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
        
        try:
            # Create pipeline
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("appsink")
        except Exception as e:
            print(f"Failed to create pipeline with {self.decoder_type} decoder: {str(e)}")
            # Try different fallback approaches
            if self.decoder_type != "software":
                print(f"Falling back to software decoder...")
                self.decoder_type = "software"
                pipeline_str = self._build_pipeline()
                try:
                    self.pipeline = Gst.parse_launch(pipeline_str)
                    self.appsink = self.pipeline.get_by_name("appsink")
                except Exception as e2:
                    print(f"Software decoder also failed: {str(e2)}")
                    # Try minimal pipeline without decoder (for testing)
                    print("Trying minimal pipeline for debugging...")
                    pipeline_str = self._build_minimal_pipeline()
                    self.pipeline = Gst.parse_launch(pipeline_str)
                    self.appsink = self.pipeline.get_by_name("appsink")
            else:
                # Try minimal pipeline as last resort
                print("Trying minimal pipeline for debugging...")
                pipeline_str = self._build_minimal_pipeline()
                self.pipeline = Gst.parse_launch(pipeline_str)
                self.appsink = self.pipeline.get_by_name("appsink")
        
        # Configure appsink
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("max-buffers", self.buffer_size)
        self.appsink.set_property("drop", True)
        
        # Connect signal
        self.appsink.connect("new-sample", self._on_new_sample, None)
            
    def _check_decoder_availability(self, decoder_name):
        """Check if a specific decoder is available."""
        try:
            # Try to create the decoder element
            decoder_element = Gst.ElementFactory.make(decoder_name, None)
            if decoder_element is not None:
                return True
        except Exception:
            pass
        return False

    def _build_pipeline(self) -> str:
        """Build GStreamer pipeline based on decoder type and encoding."""
        # For Jetson, try hardware decoder with fallback to software
        if self.decoder_type == "jetson":
            # Check if Jetson decoders are available
            if self._check_decoder_availability("nvv4l2decoder"):
                h264_decoder_chain = "nvv4l2decoder ! nvvidconv ! videoconvert"
                h265_decoder_chain = "nvv4l2decoder ! nvvidconv ! videoconvert"
                print("Using Jetson hardware decoder (nvv4l2decoder)")
            else:
                print("Jetson hardware decoder not available, using software decoder")
                h264_decoder_chain = "avdec_h264 ! videoconvert"
                h265_decoder_chain = "avdec_h265 ! videoconvert"
        elif self.decoder_type == "nvidia":
            # Check if NVIDIA GPU decoders are available
            if self._check_decoder_availability("nvh264dec"):
                h264_decoder_chain = "nvh264dec ! videoconvert"
                h265_decoder_chain = "nvh265dec ! videoconvert"
                print("Using NVIDIA GPU decoder (nvh264dec)")
            else:
                print("NVIDIA GPU decoder not available, using software decoder")
                h264_decoder_chain = "avdec_h264 ! videoconvert"
                h265_decoder_chain = "avdec_h265 ! videoconvert"
        elif self.decoder_type == "vaapi":
            # Check if VA-API decoders are available
            if self._check_decoder_availability("vaapih264dec"):
                h264_decoder_chain = "vaapih264dec ! videoconvert"
                h265_decoder_chain = "vaapih265dec ! videoconvert"
                print("Using VA-API decoder (vaapih264dec)")
            else:
                print("VA-API decoder not available, using software decoder")
                h264_decoder_chain = "avdec_h264 ! videoconvert"
                h265_decoder_chain = "avdec_h265 ! videoconvert"
        else:  # software
            h264_decoder_chain = "avdec_h264 ! videoconvert"
            h265_decoder_chain = "avdec_h265 ! videoconvert"
            print("Using software decoder (avdec_h264)")
        
        # Create pipeline string based on encoding
        if "h265" in self.rtsp_url.lower():
            pipeline_str = (
                f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true ! "
                f"rtph265depay ! h265parse ! {h265_decoder_chain} ! "
                "video/x-raw,format=BGR ! appsink name=appsink"
            )
        else:  # Default to H.264
            pipeline_str = (
                f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true ! "
                f"rtph264depay ! h264parse ! {h264_decoder_chain} ! "
                "video/x-raw,format=BGR ! appsink name=appsink"
            )
        
        return pipeline_str

    def _build_minimal_pipeline(self) -> str:
        """Build minimal pipeline that bypasses decoder issues."""
        # Simple pipeline that should work with basic GStreamer
        pipeline_str = (
            f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true ! "
            "rtph264depay ! h264parse ! queue ! decodebin ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=appsink"
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
                # Convert grayscale to BGR using numpy
                frame = np.stack([frame, frame, frame], axis=-1)
            
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
                # Try fallback to software decoder if hardware decoder fails
                if self.decoder_type == "jetson":
                    print(f"Hardware decoder failed, trying software decoder...")
                    self.decoder_type = "software"
                    self._create_pipeline()
                    ret = self.pipeline.set_state(Gst.State.PLAYING)
                    if ret == Gst.StateChangeReturn.FAILURE:
                        raise RuntimeError(f"Failed to start {self.stream_type} stream pipeline with software decoder")
                else:
                    raise RuntimeError(f"Failed to start {self.stream_type} stream pipeline")
            
            # Wait for pipeline to be ready (with timeout)
            timeout_ns = 10 * 1000000000  # 10 seconds in nanoseconds
            ret = self.pipeline.get_state(timeout_ns)
            if ret[0] != Gst.StateChangeReturn.SUCCESS:
                print(f"Warning: Pipeline state change timed out or failed for {self.stream_type} stream")
                # Don't raise an error here, allow the pipeline to continue trying
            
            print(f"Started {self.stream_type} stream from {self.rtsp_url} using {self.decoder_type} decoder")
    
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


class RTSPCameraNode:
    """
    ROS 1 Node for generic RTSP Camera streaming.
    
    This node publishes camera images and camera info to ROS topics.
    """
    
    def __init__(self, stream_type: str = "main"):
        """
        Initialize the camera node.
        
        Args:
            stream_type: Type of stream ("main" or "sub")
        """
        rospy.init_node(f'rtsp_cam_{stream_type}_node', anonymous=True)
        
        self.stream_type = stream_type
        self.camera_streamer = None
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize publishers
        self._init_publishers()
        
        # Initialize camera streamer
        self._init_camera_streamer()
        
        # Create timer for publishing using configured frame rate
        frame_rate = rospy.get_param('~frame_rate', 30)
        self.timer = rospy.Timer(rospy.Duration(1.0/frame_rate), self._publish_frame)
        
        rospy.loginfo(f"RTSP Camera {stream_type} node initialized with {frame_rate} FPS")
    
    def _declare_parameters(self):
        """Declare ROS parameters."""
        # Camera connection parameters
        self.camera_ip = rospy.get_param('~camera_ip', '192.168.0.18')
        self.username = rospy.get_param('~username', 'admin')
        self.password = rospy.get_param('~password', 'admin')
        self.camera_port = rospy.get_param('~camera_port', 554)
        
        # Stream URLs
        self.main_stream_url = rospy.get_param('~main_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/main/av_stream')
        self.sub_stream_url = rospy.get_param('~sub_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/sub/av_stream')
        
        # Stream configuration
        self.main_stream_enabled = rospy.get_param('~main_stream_enabled', True)
        self.sub_stream_enabled = rospy.get_param('~sub_stream_enabled', True)
        self.encoding = rospy.get_param('~encoding', 'h264')
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        # Stream-specific resolution parameters
        if self.stream_type == "main":
            self.resolution_width = rospy.get_param('~main_resolution_width', rospy.get_param('~resolution_width', 2560))
            self.resolution_height = rospy.get_param('~main_resolution_height', rospy.get_param('~resolution_height', 1440))
        else:  # sub stream
            self.resolution_width = rospy.get_param('~sub_resolution_width', 640)
            self.resolution_height = rospy.get_param('~sub_resolution_height', 480)
        
        # Camera info parameters
        self.camera_name = rospy.get_param('~camera_name', 'rtsp_camera')
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera_link')
        
        # Decoder parameters
        self.decoder_type = rospy.get_param('~decoder_type', 'software')
        
        # Performance parameters
        self.buffer_size = rospy.get_param('~buffer_size', 1)
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.timeout_ms = rospy.get_param('~timeout_ms', 5000)
        
        # GStreamer pipeline parameters
        self.gst_pipeline_main = rospy.get_param('~gst_pipeline_main', '')
        self.gst_pipeline_sub = rospy.get_param('~gst_pipeline_sub', '')
        
    def _init_publishers(self):
        """Initialize ROS publishers."""
        queue_size = self.queue_size
        
        # Image publisher
        self.image_pub = rospy.Publisher(
            'image_raw',
            Image,
            queue_size=queue_size
        )
        
        # Camera info publisher
        self.camera_info_pub = rospy.Publisher(
            'camera_info',
            CameraInfo,
            queue_size=queue_size
        )
        
    def _init_camera_streamer(self):
        """Initialize the camera streamer."""
        # Get stream URL based on stream type
        if self.stream_type == "main":
            rtsp_url = self.main_stream_url
            stream_enabled = self.main_stream_enabled
        else:
            rtsp_url = self.sub_stream_url
            stream_enabled = self.sub_stream_enabled
        
        # Check if stream is enabled
        if not stream_enabled:
            rospy.logwarn(f"{self.stream_type} stream is disabled in configuration")
            return
        
        # Get performance parameters
        buffer_size = self.buffer_size
        decoder_type = self.decoder_type
        
        # Get GStreamer pipeline configuration
        if self.stream_type == "main":
            gst_pipeline = self.gst_pipeline_main
        else:
            gst_pipeline = self.gst_pipeline_sub
        
        # Use custom pipeline if specified, otherwise use empty string (will use default)
        gst_pipeline = gst_pipeline if gst_pipeline else None
        
        # Create camera streamer
        self.camera_streamer = RTSPCameraStreamer(rtsp_url, self.stream_type, buffer_size, gst_pipeline, decoder_type)
        
        # Set frame callback
        self.camera_streamer.set_frame_callback(self._on_new_frame)
        
        # Start streaming
        try:
            self.camera_streamer.start()
            rospy.loginfo(f"Started {self.stream_type} stream")
        except Exception as e:
            rospy.logerr(f"Failed to start {self.stream_type} stream: {str(e)}")
    
    def _on_new_frame(self, frame):
        """Callback for new frame from camera streamer."""
        # Store frame for publishing
        self.current_frame = frame
    
    def _publish_frame(self, event):
        """Publish current frame to ROS topic."""
        if hasattr(self, 'current_frame') and self.current_frame is not None:
            try:
                # Convert frame to ROS message
                # Convert numpy array to ROS Image message manually
                ros_image = Image()
                if self.current_frame is not None:
                    ros_image.height = self.current_frame.shape[0]
                    ros_image.width = self.current_frame.shape[1]
                    ros_image.encoding = "bgr8"
                    ros_image.is_bigendian = False
                    ros_image.step = self.current_frame.shape[1] * 3  # 3 bytes per pixel for BGR
                    ros_image.data = self.current_frame.tobytes()
                else:
                    return  # No frame available
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = self.camera_frame_id
                
                # Publish image
                self.image_pub.publish(ros_image)
                
                # Publish camera info
                self._publish_camera_info()
                
            except Exception as e:
                rospy.logerr(f"Error publishing frame: {str(e)}")
    
    def _publish_camera_info(self):
        """Publish camera info message."""
        camera_info = CameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.frame_id = self.camera_frame_id
        
        # Set camera parameters from configuration
        camera_info.width = self.resolution_width
        camera_info.height = self.resolution_height
        
        # Set distortion model (you may need to calibrate your camera)
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set camera matrix (example values - you should calibrate your camera)
        camera_info.K = [
            camera_info.width * 0.8, 0.0, camera_info.width / 2.0,
            0.0, camera_info.height * 0.8, camera_info.height / 2.0,
            0.0, 0.0, 1.0
        ]
        
        # Set rectification matrix (identity for monocular camera)
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Set projection matrix
        camera_info.P = [
            camera_info.K[0], 0.0, camera_info.K[2], 0.0,
            0.0, camera_info.K[4], camera_info.K[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Publish camera info
        self.camera_info_pub.publish(camera_info)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if self.camera_streamer:
            self.camera_streamer.stop()


def main():
    """Main function for the camera node."""
    try:
        # Create camera node (default to main stream)
        camera_node = RTSPCameraNode("main")
        
        # Spin the node
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Camera node interrupted by user")
    except Exception as e:
        print(f"Error in camera node: {str(e)}")
    finally:
        # Cleanup
        if 'camera_node' in locals():
            camera_node.destroy_node()


if __name__ == '__main__':
    main() 