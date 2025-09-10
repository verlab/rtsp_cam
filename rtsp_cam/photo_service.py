#!/usr/bin/env python3
"""
Photo Service Module for Generic RTSP Camera

This module provides photo capture functionality for RTSP cameras with snapshot capabilities.
It includes both HTTP-based and RTSP-based photo capture methods.
"""

import cv2
import os
import time
import requests
from typing import Optional, Tuple
from datetime import datetime

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PhotoCaptureService:
    """
    Photo capture service for RTSP cameras with snapshot capabilities.
    
    This class provides methods to capture photos using both HTTP and RTSP interfaces.
    """
    
    def __init__(self, ip: str, username: str = "admin", password: str = "admin", 
                 http_port: int = 80, rtsp_port: int = 554,
                 resolution_width: int = 1920, resolution_height: int = 1080,
                 photo_quality: int = 95, main_stream_url: str = None):
        """
        Initialize photo capture service.
        
        Args:
            ip: Camera IP address
            username: Camera username
            password: Camera password
            http_port: HTTP port for photo capture
            rtsp_port: RTSP port for video stream
            resolution_width: Photo width resolution
            resolution_height: Photo height resolution
            photo_quality: Photo quality (0-100)
            main_stream_url: Main stream URL for RTSP capture
        """
        self.ip = ip
        self.username = username
        self.password = password
        self.http_port = http_port
        self.rtsp_port = rtsp_port
        self.resolution_width = resolution_width
        self.resolution_height = resolution_height
        self.photo_quality = photo_quality
        self.main_stream_url = main_stream_url
        
        # Create session for HTTP requests
        self.session = requests.Session()
        self.session.auth = (username, password)
        
        # RTSP capture
        self.rtsp_cap = None
        
    def capture_photo_http(self, save_path: str, filename: Optional[str] = None) -> Tuple[bool, str]:
        """
        Capture photo using HTTP interface.
        
        Args:
            save_path: Directory to save the photo
            filename: Optional filename (if None, auto-generate)
            
        Returns:
            Tuple of (success, message)
        """
        try:
            # Create save directory if it doesn't exist
            os.makedirs(save_path, exist_ok=True)
            
            # Generate filename if not provided
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"jidetech_photo_{timestamp}.jpg"
            
            # Full path for the photo
            photo_path = os.path.join(save_path, filename)
            
            # Try multiple HTTP snapshot URLs (different cameras use different endpoints)
            snapshot_urls = [
                # Jidetech/Dahua style URLs
                f"http://{self.ip}:{self.http_port}/cgi-bin/snapshot.cgi",
                f"http://{self.ip}:{self.http_port}/snapshot.cgi",
                f"http://{self.ip}:{self.http_port}/cgi-bin/snapshot.cgi?channel=0",
                f"http://{self.ip}:{self.http_port}/form/getSnapshot",
                f"http://{self.ip}:{self.http_port}/form/getSnapshot.cgi",
                
                # Alternative formats
                f"http://{self.ip}:{self.http_port}/jpg/image.jpg",
                f"http://{self.ip}:{self.http_port}/tmpfs/auto.jpg",
                f"http://{self.ip}:{self.http_port}/image.jpg",
                f"http://{self.ip}:{self.http_port}/snapshot.jpg",
                
                # CGI variations
                f"http://{self.ip}:{self.http_port}/cgi-bin/hi3510/snap.cgi",
                f"http://{self.ip}:{self.http_port}/webcapture.jpg",
                f"http://{self.ip}:{self.http_port}/axis-cgi/jpg/image.cgi",
            ]
            
            # Parameters for photo capture using configured values
            params = {
                'channel': '0',
                'quality': str(self.photo_quality),
                'width': str(self.resolution_width),
                'height': str(self.resolution_height)
            }
            
            # Try each snapshot URL until one works
            for snapshot_url in snapshot_urls:
                try:
                    print(f"🔍 Trying snapshot URL: {snapshot_url}")
                    
                    # Try with parameters first
                    response = self.session.get(snapshot_url, params=params, timeout=10)
                    
                    if response.status_code == 200 and len(response.content) > 1000:  # Valid image should be > 1KB
                        # Save photo
                        with open(photo_path, 'wb') as f:
                            f.write(response.content)
                        
                        # Verify file was written correctly
                        if os.path.exists(photo_path) and os.path.getsize(photo_path) > 1000:
                            print(f"✅ Photo captured successfully using: {snapshot_url}")
                            return True, f"Photo captured successfully: {photo_path} ({len(response.content)} bytes)"
                        else:
                            print(f"⚠️  Photo file is too small or empty: {os.path.getsize(photo_path) if os.path.exists(photo_path) else 0} bytes")
                    
                    # If parameters failed, try without parameters
                    if response.status_code != 200 or len(response.content) <= 1000:
                        print(f"   Trying without parameters...")
                        response = self.session.get(snapshot_url, timeout=10)
                        
                        if response.status_code == 200 and len(response.content) > 1000:
                            # Save photo
                            with open(photo_path, 'wb') as f:
                                f.write(response.content)
                            
                            # Verify file was written correctly
                            if os.path.exists(photo_path) and os.path.getsize(photo_path) > 1000:
                                print(f"✅ Photo captured successfully using: {snapshot_url} (no params)")
                                return True, f"Photo captured successfully: {photo_path} ({len(response.content)} bytes)"
                    
                    print(f"   ❌ Failed: HTTP {response.status_code}, {len(response.content)} bytes")
                    
                except Exception as url_error:
                    print(f"   ❌ Error with {snapshot_url}: {url_error}")
                    continue
            
            # If all HTTP methods failed, return error
            return False, f"Failed to capture photo: All HTTP snapshot URLs failed"
                
        except Exception as e:
            return False, f"Error capturing photo via HTTP: {str(e)}"
    
    def capture_photo_rtsp(self, save_path: str, filename: Optional[str] = None) -> Tuple[bool, str]:
        """
        Capture photo using RTSP stream.
        
        Args:
            save_path: Directory to save the photo
            filename: Optional filename (if None, auto-generate)
            
        Returns:
            Tuple of (success, message)
        """
        try:
            # Create save directory if it doesn't exist
            os.makedirs(save_path, exist_ok=True)
            
            # Generate filename if not provided
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"jidetech_photo_{timestamp}.jpg"
            
            # Full path for the photo
            photo_path = os.path.join(save_path, filename)
            
            # Use configured main stream URL or build default
            if self.main_stream_url:
                rtsp_url = self.main_stream_url
            else:
                rtsp_url = f"rtsp://{self.username}:{self.password}@{self.ip}:{self.rtsp_port}/h264/ch1/main/av_stream"
            
            # Open RTSP stream
            cap = cv2.VideoCapture(rtsp_url)
            
            if not cap.isOpened():
                return False, "Failed to open RTSP stream"
            
            # Read frame
            ret, frame = cap.read()
            
            if ret:
                # Resize frame to configured resolution if needed
                if frame.shape[1] != self.resolution_width or frame.shape[0] != self.resolution_height:
                    frame = cv2.resize(frame, (self.resolution_width, self.resolution_height))
                
                # Save frame as photo with configured quality
                cv2.imwrite(photo_path, frame, [cv2.IMWRITE_JPEG_QUALITY, self.photo_quality])
                cap.release()
                return True, f"Photo captured successfully: {photo_path}"
            else:
                cap.release()
                return False, "Failed to read frame from RTSP stream"
                
        except Exception as e:
            return False, f"Error capturing photo via RTSP: {str(e)}"
    
    def capture_photo_auto(self, save_path: str, filename: Optional[str] = None, 
                          prefer_http: bool = True) -> Tuple[bool, str]:
        """
        Capture photo using automatic method selection.
        
        Args:
            save_path: Directory to save the photo
            filename: Optional filename (if None, auto-generate)
            prefer_http: Whether to prefer HTTP over RTSP
            
        Returns:
            Tuple of (success, message)
        """
        if prefer_http:
            # Try HTTP first, then RTSP
            success, message = self.capture_photo_http(save_path, filename)
            if success:
                return success, message
            
            # Fallback to RTSP
            return self.capture_photo_rtsp(save_path, filename)
        else:
            # Try RTSP first, then HTTP
            success, message = self.capture_photo_rtsp(save_path, filename)
            if success:
                return success, message
            
            # Fallback to HTTP
            return self.capture_photo_http(save_path, filename)


class JidetechPhotoService(Node):
    """
    ROS 2 Node for Jidetech Camera photo capture service.
    
    This node provides a ROS service to capture photos from the camera.
    """
    
    def __init__(self):
        """Initialize the photo service node."""
        super().__init__('jidetech_photo_service')
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize photo capture service
        self._init_photo_service()
        
        # Initialize ROS service
        self._init_service()
        
        # Initialize bridge for image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info("Jidetech Photo Service initialized")
    
    def _declare_parameters(self):
        """Declare ROS parameters."""
        # Camera connection parameters
        self.declare_parameter('camera_ip', '192.168.0.18')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')
        self.declare_parameter('http_port', 80)
        self.declare_parameter('rtsp_port', 554)
        
        # Stream URLs
        self.declare_parameter('main_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/main/av_stream')
        
        # Resolution parameters
        self.declare_parameter('resolution_width', 1920)
        self.declare_parameter('resolution_height', 1080)
        
        # Photo capture parameters
        self.declare_parameter('photo_save_path', '/tmp/jidetech_photos')
        self.declare_parameter('photo_format', 'jpg')
        self.declare_parameter('photo_quality', 95)
        self.declare_parameter('prefer_http', True)
        
    def _init_photo_service(self):
        """Initialize photo capture service."""
        camera_ip = self.get_parameter('camera_ip').value
        username = self.get_parameter('username').value
        password = self.get_parameter('password').value
        http_port = self.get_parameter('http_port').value
        rtsp_port = self.get_parameter('rtsp_port').value
        main_stream_url = self.get_parameter('main_stream_url').value
        resolution_width = self.get_parameter('resolution_width').value
        resolution_height = self.get_parameter('resolution_height').value
        photo_quality = self.get_parameter('photo_quality').value
        
        self.photo_service = PhotoCaptureService(
            camera_ip, username, password, http_port, rtsp_port,
            resolution_width, resolution_height, photo_quality, main_stream_url
        )
    
    def _init_service(self):
        """Initialize ROS service."""
        self.photo_srv = self.create_service(
            Trigger,
            'take_photo',
            self._take_photo_callback
        )
    
    def _take_photo_callback(self, request, response):
        """
        Callback for photo capture service.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response with success status and message
        """
        try:
            # Get parameters
            save_path = self.get_parameter('photo_save_path').value
            prefer_http = self.get_parameter('prefer_http').value
            photo_format = self.get_parameter('photo_format').value
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"jidetech_photo_{timestamp}.{photo_format}"
            
            # Capture photo
            success, message = self.photo_service.capture_photo_auto(
                save_path, filename, prefer_http
            )
            
            if success:
                response.success = True
                response.message = message
                self.get_logger().info(f"Photo captured: {message}")
            else:
                response.success = False
                response.message = message
                self.get_logger().error(f"Photo capture failed: {message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error in photo capture service: {str(e)}"
            self.get_logger().error(f"Photo capture service error: {str(e)}")
        
        return response
    
    def capture_photo_from_topic(self, image_msg: Image, save_path: str, 
                                filename: Optional[str] = None) -> Tuple[bool, str]:
        """
        Capture photo from ROS image message.
        
        Args:
            image_msg: ROS Image message
            save_path: Directory to save the photo
            filename: Optional filename (if None, auto-generate)
            
        Returns:
            Tuple of (success, message)
        """
        try:
            # Create save directory if it doesn't exist
            os.makedirs(save_path, exist_ok=True)
            
            # Generate filename if not provided
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                photo_format = self.get_parameter('photo_format').value
                filename = f"jidetech_photo_{timestamp}.{photo_format}"
            
            # Full path for the photo
            photo_path = os.path.join(save_path, filename)
            
            # Convert ROS message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Get configured resolution and quality
            resolution_width = self.get_parameter('resolution_width').value
            resolution_height = self.get_parameter('resolution_height').value
            photo_quality = self.get_parameter('photo_quality').value
            
            # Resize image to configured resolution if needed
            if cv_image.shape[1] != resolution_width or cv_image.shape[0] != resolution_height:
                cv_image = cv2.resize(cv_image, (resolution_width, resolution_height))
            
            # Save image with configured quality
            cv2.imwrite(photo_path, cv_image, [cv2.IMWRITE_JPEG_QUALITY, photo_quality])
            
            return True, f"Photo captured from topic: {photo_path}"
            
        except Exception as e:
            return False, f"Error capturing photo from topic: {str(e)}"


def main(args=None):
    """Main function for the photo service node."""
    rclpy.init(args=args)
    
    try:
        # Create photo service node
        photo_node = JidetechPhotoService()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(photo_node)
        
        # Spin the node
        executor.spin()
        
    except KeyboardInterrupt:
        print("Photo service node interrupted by user")
    except Exception as e:
        print(f"Error in photo service node: {str(e)}")
    finally:
        # Cleanup
        if 'photo_node' in locals():
            photo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 