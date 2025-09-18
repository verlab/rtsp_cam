#!/usr/bin/env python3
"""
Generic RTSP Camera PTZ Controller

This module provides PTZ (Pan-Tilt-Zoom) control for RTSP cameras with PTZ capabilities.
Uses HTTP command format (configurable for different camera types): /form/setPTZCfg?command=X

Commands:
- 0: Stop
- 1: Up
- 2: Down  
- 3: Left
- 4: Right
- 5: Down-Left
- 6: Up-Left
- 7: Up-Right
- 8: Down-Right
- 9: Iris Open
- 10: Iris Close
- 11: Focus Far
- 12: Focus Near
- 13: Zoom In (Tele)
- 14: Zoom Out (Wide)
- 15: Set Speed (with panSpeed/tiltSpeed parameters)

Author: rezeck <rezeck@ufmg.br>
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import requests
from requests.auth import HTTPBasicAuth
import threading
import time
from typing import Optional, Dict, Any
import math


class RTSPCameraPTZController:
    """
    PTZ Controller using ZoneMinder command format.
    
    This class provides PTZ control functionality using the working
    HTTP command format discovered from ZoneMinder integration.
    """
    
    def __init__(self, ip: str, username: str = "admin", password: str = "admin", 
                 http_port: int = 80, timeout_ms: int = 5000):
        """
        Initialize PTZ controller.
        
        Args:
            ip: Camera IP address
            username: Camera username
            password: Camera password
            http_port: HTTP port
            timeout_ms: Request timeout in milliseconds
        """
        self.ip = ip
        self.username = username
        self.password = password
        self.http_port = http_port
        self.timeout = timeout_ms / 1000.0
        
        # Create HTTP session with authentication
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
        
        # PTZ state tracking
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_zoom = 0.0
        self.is_moving = False
        self.last_command = 0
        self.movement_start_time = 0.0
        
        # Movement tracking for position estimation
        self.pan_speed = 50  # Default speed (0-100)
        self.tilt_speed = 50
        self.zoom_speed = 50
        
        # Auto-stop timer
        self.stop_timer = None
        self.auto_stop_duration = 0.1  # Auto-stop after 100ms if no new commands
        
        # Base URL for PTZ commands
        self.base_url = f"http://{self.ip}:{self.http_port}"
        
        print(f"🎯 PTZ Controller initialized for {self.ip}")
        print(f"   Using ZoneMinder format: /form/setPTZCfg?command=X")
    
    def _send_command(self, command: int, **kwargs) -> bool:
        """
        Send PTZ command using ZoneMinder format.
        
        Args:
            command: Command number (0-15)
            **kwargs: Additional parameters (e.g., panSpeed, tiltSpeed)
            
        Returns:
            True if command sent successfully
        """
        try:
            url = f"{self.base_url}/form/setPTZCfg"
            params = {"command": str(command)}
            
            # Add additional parameters
            for key, value in kwargs.items():
                params[key] = str(value)
            
            print(f"🌐 Sending HTTP request: {url} with params: {params}")
            
            response = self.session.get(url, params=params, timeout=self.timeout)
            
            success = response.status_code == 200
            print(f"📡 HTTP Response: {response.status_code} - {'Success' if success else 'Failed'}")
            
            if success:
                self.last_command = command
                if command != 0:  # Not a stop command
                    self.is_moving = True
                    self.movement_start_time = time.time()
                else:
                    self.is_moving = False
            
            return success
            
        except Exception as e:
            print(f"❌ PTZ command failed: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop all PTZ movement."""
        return self._send_command(0)
    
    def move_up(self, duration: Optional[float] = None) -> bool:
        """Move camera up."""
        success = self._send_command(1)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_down(self, duration: Optional[float] = None) -> bool:
        """Move camera down."""
        success = self._send_command(2)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_left(self, duration: Optional[float] = None) -> bool:
        """Move camera left."""
        success = self._send_command(3)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_right(self, duration: Optional[float] = None) -> bool:
        """Move camera right."""
        success = self._send_command(4)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_down_left(self, duration: Optional[float] = None) -> bool:
        """Move camera down-left diagonal."""
        success = self._send_command(5)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_up_left(self, duration: Optional[float] = None) -> bool:
        """Move camera up-left diagonal."""
        success = self._send_command(6)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_up_right(self, duration: Optional[float] = None) -> bool:
        """Move camera up-right diagonal."""
        success = self._send_command(7)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def move_down_right(self, duration: Optional[float] = None) -> bool:
        """Move camera down-right diagonal."""
        success = self._send_command(8)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def iris_open(self, duration: Optional[float] = None) -> bool:
        """Open iris."""
        success = self._send_command(9)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def iris_close(self, duration: Optional[float] = None) -> bool:
        """Close iris."""
        success = self._send_command(10)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def focus_far(self, duration: Optional[float] = None) -> bool:
        """Focus far."""
        success = self._send_command(11)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def focus_near(self, duration: Optional[float] = None) -> bool:
        """Focus near."""
        success = self._send_command(12)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def zoom_in(self, duration: Optional[float] = None) -> bool:
        """Zoom in (tele)."""
        success = self._send_command(13)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def zoom_out(self, duration: Optional[float] = None) -> bool:
        """Zoom out (wide)."""
        success = self._send_command(14)
        if success and duration:
            self._schedule_auto_stop(duration)
        return success
    
    def set_speed(self, pan_speed: int = 50, tilt_speed: int = 50) -> bool:
        """
        Set PTZ movement speed.
        
        Args:
            pan_speed: Pan speed (0-100)
            tilt_speed: Tilt speed (0-100)
            
        Returns:
            True if speed set successfully
        """
        success = self._send_command(15, panSpeed=pan_speed, tiltSpeed=tilt_speed)
        if success:
            self.pan_speed = pan_speed
            self.tilt_speed = tilt_speed
        return success
    
    def store_preset(self, preset_num: int) -> bool:
        """
        Store current position as preset.
        
        Args:
            preset_num: Preset number (1-255)
            
        Returns:
            True if preset stored successfully
        """
        try:
            url = f"{self.base_url}/form/presetSet"
            params = {
                "existFlag": 1,
                "flag": 3,
                "language": "en",
                "presetNum": preset_num
            }
            
            response = self.session.get(url, params=params, timeout=self.timeout)
            return response.status_code == 200
            
        except Exception as e:
            print(f"❌ Store preset failed: {e}")
            return False
    
    def recall_preset(self, preset_num: int) -> bool:
        """
        Recall stored preset position.
        
        Args:
            preset_num: Preset number (1-255)
            
        Returns:
            True if preset recalled successfully
        """
        try:
            url = f"{self.base_url}/form/presetSet"
            params = {
                "existFlag": 1,
                "flag": 4,
                "language": "en",
                "presetNum": preset_num
            }
            
            response = self.session.get(url, params=params, timeout=self.timeout)
            return response.status_code == 200
            
        except Exception as e:
            print(f"❌ Recall preset failed: {e}")
            return False
    
    def start_patrol(self) -> bool:
        """Start horizontal patrol."""
        try:
            url = f"{self.base_url}/form/setSpecialFunc"
            params = {"command": 1, "flag": 2}
            
            response = self.session.get(url, params=params, timeout=self.timeout)
            return response.status_code == 200
            
        except Exception as e:
            print(f"❌ Start patrol failed: {e}")
            return False
    
    def _schedule_auto_stop(self, duration: float):
        """Schedule automatic stop after duration."""
        if self.stop_timer:
            self.stop_timer.cancel()
        
        self.stop_timer = threading.Timer(duration, self.stop)
        self.stop_timer.start()
    
    def process_twist_command(self, twist: Twist) -> bool:
        """
        Process ROS Twist message for PTZ control.
        
        Args:
            twist: ROS Twist message
            
        Returns:
            True if command processed successfully
        """
        # Extract velocities
        linear_x = twist.linear.x   # Forward/backward (tilt)
        linear_y = twist.linear.y   # Left/right (pan)  
        linear_z = twist.linear.z   # Up/down (zoom)
        
        print(f"🎯 Processing Twist: x={linear_x:.2f}, y={linear_y:.2f}, z={linear_z:.2f}")
        
        # Deadzone threshold
        deadzone = 0.1
        
        # Determine movement command based on velocities
        if abs(linear_x) < deadzone and abs(linear_y) < deadzone and abs(linear_z) < deadzone:
            # Stop command
            print("🛑 Sending STOP command")
            return self.stop()
        
        # Handle zoom first (highest priority)
        if abs(linear_z) > deadzone:
            if linear_z > 0:
                return self.zoom_in(self.auto_stop_duration)
            else:
                return self.zoom_out(self.auto_stop_duration)
        
        # Handle pan/tilt combinations
        if abs(linear_x) > deadzone and abs(linear_y) > deadzone:
            # Diagonal movement
            if linear_x > 0 and linear_y > 0:
                return self.move_up_right(self.auto_stop_duration)
            elif linear_x > 0 and linear_y < 0:
                return self.move_up_left(self.auto_stop_duration)
            elif linear_x < 0 and linear_y > 0:
                return self.move_down_right(self.auto_stop_duration)
            else:  # linear_x < 0 and linear_y < 0
                return self.move_down_left(self.auto_stop_duration)
        
        # Handle single-axis movement
        elif abs(linear_x) > deadzone:
            # Tilt movement
            if linear_x > 0:
                return self.move_up(self.auto_stop_duration)
            else:
                return self.move_down(self.auto_stop_duration)
        
        elif abs(linear_y) > deadzone:
            # Pan movement
            if linear_y > 0:
                print("➡️ Sending MOVE RIGHT command")
                return self.move_right(self.auto_stop_duration)
            else:
                print("⬅️ Sending MOVE LEFT command")
                return self.move_left(self.auto_stop_duration)
        
        return False
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current PTZ status.
        
        Returns:
            Dictionary with PTZ status information
        """
        return {
            "pan": self.current_pan,
            "tilt": self.current_tilt,
            "zoom": self.current_zoom,
            "is_moving": self.is_moving,
            "last_command": self.last_command,
            "pan_speed": self.pan_speed,
            "tilt_speed": self.tilt_speed,
            "connected": True  # Assume connected if we can create the object
        }


class RTSPCameraPTZNode:
    """
    ROS 1 Node for RTSP PTZ control.
    
    This node provides PTZ control via ROS topics and services.
    """
    
    def __init__(self):
        """Initialize the PTZ node."""
        rospy.init_node('rtsp_ptz_controller', anonymous=True)
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize PTZ controller
        self._init_ptz_controller()
        
        # Initialize ROS interfaces
        self._init_topics()
        
        # Start status publishing timer
        self._init_status_timer()
        
        rospy.loginfo("RTSP Camera PTZ Controller initialized")
    
    def _declare_parameters(self):
        """Declare ROS parameters."""
        # Camera connection parameters
        self.camera_ip = rospy.get_param('~camera_ip', '192.168.0.18')
        self.username = rospy.get_param('~username', 'admin')
        self.password = rospy.get_param('~password', 'admin')
        self.http_port = rospy.get_param('~http_port', 80)
        self.timeout_ms = rospy.get_param('~timeout_ms', 5000)
        
        # PTZ control parameters
        self.default_pan_speed = rospy.get_param('~default_pan_speed', 50)
        self.default_tilt_speed = rospy.get_param('~default_tilt_speed', 50)
        self.auto_stop_duration = rospy.get_param('~auto_stop_duration', 0.1)
        
        # Topic names
        self.ptz_cmd_topic = rospy.get_param('~ptz_cmd_topic', 'ptz_cmd')
        self.ptz_status_topic = rospy.get_param('~ptz_status_topic', 'ptz_status')
        self.joint_states_topic = rospy.get_param('~joint_states_topic', 'joint_states')
        
        # Enable/disable flags
        self.enable_ptz_control = rospy.get_param('~enable_ptz_control', True)
        self.enable_status_publishing = rospy.get_param('~enable_status_publishing', True)
        self.status_publish_rate = rospy.get_param('~status_publish_rate', 10.0)
    
    def _init_ptz_controller(self):
        """Initialize PTZ controller."""
        camera_ip = self.camera_ip
        username = self.username
        password = self.password
        http_port = self.http_port
        timeout_ms = self.timeout_ms
        
        self.ptz_controller = RTSPCameraPTZController(
            camera_ip, username, password, http_port, timeout_ms
        )
        
        # Set default speeds
        default_pan_speed = self.default_pan_speed
        default_tilt_speed = self.default_tilt_speed
        auto_stop_duration = self.auto_stop_duration
        
        self.ptz_controller.set_speed(default_pan_speed, default_tilt_speed)
        self.ptz_controller.auto_stop_duration = auto_stop_duration
    
    def _init_topics(self):
        """Initialize ROS topics."""
        enable_ptz = self.enable_ptz_control
        
        if enable_ptz:
            # PTZ command subscriber
            ptz_cmd_topic = self.ptz_cmd_topic
            self.ptz_cmd_sub = rospy.Subscriber(
                ptz_cmd_topic,
                Twist,
                self._ptz_cmd_callback,
                queue_size=10
            )
        
        # Status publishers
        enable_status = self.enable_status_publishing
        if enable_status:
            ptz_status_topic = self.ptz_status_topic
            joint_states_topic = self.joint_states_topic
            
            self.ptz_status_pub = rospy.Publisher(ptz_status_topic, String, queue_size=10)
            self.joint_states_pub = rospy.Publisher(joint_states_topic, JointState, queue_size=10)
    
    def _init_status_timer(self):
        """Initialize status publishing timer."""
        enable_status = self.enable_status_publishing
        if enable_status:
            status_rate = self.status_publish_rate
            timer_period = 1.0 / status_rate
            
            self.status_timer = rospy.Timer(rospy.Duration(timer_period), self._publish_status)
    
    def _ptz_cmd_callback(self, msg):
        """
        Callback for PTZ command messages.
        
        Args:
            msg: ROS Twist message with PTZ commands
        """
        try:
            enable_ptz = self.enable_ptz_control
            if not enable_ptz:
                rospy.logwarn("PTZ control is disabled")
                return
            
            rospy.loginfo(f"Received PTZ command: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
            
            # Process twist command
            success = self.ptz_controller.process_twist_command(msg)
            
            if success:
                rospy.loginfo("PTZ command processed successfully")
            else:
                rospy.logwarn("Failed to process PTZ command")
                
        except Exception as e:
            rospy.logerr(f"Error processing PTZ command: {str(e)}")
    
    def _publish_status(self, event):
        """Publish PTZ status information."""
        try:
            # Get PTZ status
            status = self.ptz_controller.get_status()
            
            # Publish status string
            status_msg = String()
            status_msg.data = f"pan:{status['pan']:.2f},tilt:{status['tilt']:.2f},zoom:{status['zoom']:.2f},moving:{status['is_moving']},cmd:{status['last_command']}"
            self.ptz_status_pub.publish(status_msg)
            
            # Publish joint states
            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = ['pan_joint', 'tilt_joint', 'zoom_joint']
            joint_msg.position = [
                math.radians(status['pan']),
                math.radians(status['tilt']),
                status['zoom']
            ]
            joint_msg.velocity = [0.0, 0.0, 0.0]
            joint_msg.effort = [0.0, 0.0, 0.0]
            
            self.joint_states_pub.publish(joint_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing PTZ status: {str(e)}")


def main():
    """Main function for the PTZ controller node."""
    try:
        # Create PTZ controller node
        ptz_node = RTSPCameraPTZNode()
        
        # Spin the node
        rospy.spin()
        
    except KeyboardInterrupt:
        print("PTZ controller node interrupted by user")
    except Exception as e:
        print(f"Error in PTZ controller node: {str(e)}")
    finally:
        # Cleanup
        if 'ptz_node' in locals():
            ptz_node.destroy_node()


if __name__ == '__main__':
    main() 