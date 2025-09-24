#!/usr/bin/env python3
"""
Photo Service Module for Generic RTSP Camera

This module provides photo capture functionality for RTSP cameras with snapshot capabilities.
It includes both HTTP-based and RTSP-based photo capture methods.
"""

try:
    import cv2
except ImportError as e:
    print(f"OpenCV import failed: {e}")
    print("Using fallback image processing...")
    cv2 = None
import os
import time
import requests
from typing import Optional, Tuple
from datetime import datetime

# ROS 1 imports
import rospy
from std_srvs.srv import Trigger, TriggerResponse
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
            if cv2 is None:
                rospy.logwarn("OpenCV not available - RTSP photo capture disabled")
                return TriggerResponse(success=False, message="OpenCV not available")
            
            cap = cv2.VideoCapture(rtsp_url)
            
            if not cap.isOpened():
                return False, "Failed to open RTSP stream"
            
            # Read frame
            ret, frame = cap.read()
            
            if ret:
                # Resize frame to configured resolution if needed
                if cv2 is not None and frame.shape[1] != self.resolution_width or frame.shape[0] != self.resolution_height:
                    frame = cv2.resize(frame, (self.resolution_width, self.resolution_height))
                
                # Save frame as photo with configured quality
                if cv2 is not None:
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


class RTSPPhotoService:
    """
    ROS 1 Node for RTSP Camera photo capture service.
    
    This node provides a ROS service to capture photos from the camera.
    """
    
    def __init__(self):
        """Initialize the photo service node."""
        rospy.init_node('rtsp_photo_service', anonymous=True)
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize photo capture service
        self._init_photo_service()
        
        # Initialize ROS service
        self._init_service()
        
        # Initialize bridge for image conversion
        self.bridge = CvBridge()
        
        rospy.loginfo("RTSP Photo Service initialized")
    
    def _declare_parameters(self):
        """Declare ROS parameters."""
        # Camera connection parameters
        self.camera_ip = rospy.get_param('~camera_ip', '192.168.0.18')
        self.username = rospy.get_param('~username', 'admin')
        self.password = rospy.get_param('~password', 'admin')
        self.http_port = rospy.get_param('~http_port', 80)
        self.rtsp_port = rospy.get_param('~rtsp_port', 554)
        
        # Stream URLs
        self.main_stream_url = rospy.get_param('~main_stream_url', 
                              'rtsp://admin:admin@192.168.0.18:554/h264/ch1/main/av_stream')
        
        # Resolution parameters
        self.resolution_width = rospy.get_param('~resolution_width', 1920)
        self.resolution_height = rospy.get_param('~resolution_height', 1080)
        
        # Photo capture parameters
        self.photo_save_path = rospy.get_param('~photo_save_path', '/tmp/rtsp_cam_photos')
        self.photo_format = rospy.get_param('~photo_format', 'jpg')
        self.photo_quality = rospy.get_param('~photo_quality', 95)
        self.prefer_http = rospy.get_param('~prefer_http', True)
        
    def _init_photo_service(self):
        """Initialize photo capture service."""
        camera_ip = self.camera_ip
        username = self.username
        password = self.password
        http_port = self.http_port
        rtsp_port = self.rtsp_port
        main_stream_url = self.main_stream_url
        resolution_width = self.resolution_width
        resolution_height = self.resolution_height
        photo_quality = self.photo_quality
        
        self.photo_service = PhotoCaptureService(
            camera_ip, username, password, http_port, rtsp_port,
            resolution_width, resolution_height, photo_quality, main_stream_url
        )
    
    def _init_service(self):
        """Initialize ROS service."""
        self.photo_srv = rospy.Service(
            'take_photo',
            Trigger,
            self._take_photo_callback
        )
    
    def _take_photo_callback(self, request):
        """
        Callback for photo capture service.
        
        Args:
            request: Service request
            
        Returns:
            Service response with success status and message
        """
        try:
            # Get parameters
            save_path = self.photo_save_path
            prefer_http = self.prefer_http
            photo_format = self.photo_format
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"rtsp_photo_{timestamp}.{photo_format}"
            
            # Capture photo
            success, message = self.photo_service.capture_photo_auto(
                save_path, filename, prefer_http
            )
            
            if success:
                rospy.loginfo(f"Photo captured: {message}")
                return TriggerResponse(success=True, message=message)
            else:
                rospy.logerr(f"Photo capture failed: {message}")
                return TriggerResponse(success=False, message=message)
            
        except Exception as e:
            error_msg = f"Error in photo capture service: {str(e)}"
            rospy.logerr(f"Photo capture service error: {str(e)}")
            return TriggerResponse(success=False, message=error_msg)
    
    def capture_photo_from_topic(self, image_msg, save_path, filename=None):
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
                photo_format = self.photo_format
                filename = f"rtsp_photo_{timestamp}.{photo_format}"
            
            # Full path for the photo
            photo_path = os.path.join(save_path, filename)
            
            # Convert ROS message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Get configured resolution and quality
            resolution_width = self.resolution_width
            resolution_height = self.resolution_height
            photo_quality = self.photo_quality
            
            # Resize image to configured resolution if needed
            if cv2 is not None and cv_image.shape[1] != resolution_width or cv_image.shape[0] != resolution_height:
                cv_image = cv2.resize(cv_image, (resolution_width, resolution_height))
            
            # Save image with configured quality
            if cv2 is not None:
                cv2.imwrite(photo_path, cv_image, [cv2.IMWRITE_JPEG_QUALITY, photo_quality])
            
            return True, f"Photo captured from topic: {photo_path}"
            
        except Exception as e:
            return False, f"Error capturing photo from topic: {str(e)}"


def main():
    """Main function for the photo service node."""
    try:
        # Create photo service node
        photo_node = RTSPPhotoService()
        
        # Spin the node
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Photo service node interrupted by user")
    except Exception as e:
        print(f"Error in photo service node: {str(e)}")
    finally:
        # Cleanup
        if 'photo_node' in locals():
            photo_node.destroy_node()


if __name__ == '__main__':
    main() 