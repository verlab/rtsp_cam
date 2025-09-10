#!/usr/bin/env python3
"""
PTZ Status Monitor for Jidetech Camera

This script monitors PTZ status messages and displays real-time information
about the camera's PTZ state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from datetime import datetime


class PTZStatusMonitor(Node):
    """Monitor PTZ status messages."""
    
    def __init__(self):
        super().__init__('ptz_status_monitor')
        
        # Subscribe to PTZ status topics
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/camera/ptz_status',
            self.joint_state_callback,
            10
        )
        
        self.status_string_sub = self.create_subscription(
            String,
            '/camera/ptz_status_string',
            self.status_string_callback,
            10
        )
        
        # Subscribe to PTZ commands to see what's being sent
        self.ptz_cmd_sub = self.create_subscription(
            Twist,
            '/camera/ptz_cmd',
            self.ptz_cmd_callback,
            10
        )
        
        # Status tracking
        self.last_joint_state = None
        self.last_status_string = None
        self.last_ptz_cmd = None
        self.last_update_time = None
        
        # Create timer for status display
        self.timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info("PTZ Status Monitor initialized")
        self.print_header()
    
    def print_header(self):
        """Print monitor header."""
        print("\n" + "="*80)
        print("                        PTZ STATUS MONITOR")
        print("="*80)
        print("Monitoring topics:")
        print("  - /camera/ptz_status (JointState)")
        print("  - /camera/ptz_status_string (String)")
        print("  - /camera/ptz_cmd (Twist)")
        print("="*80)
        print()
    
    def joint_state_callback(self, msg):
        """Callback for JointState messages."""
        self.last_joint_state = msg
        self.last_update_time = datetime.now()
        
        # Log detailed joint state info
        if len(msg.name) >= 3 and len(msg.position) >= 3:
            self.get_logger().debug(
                f"Joint State - Pan: {msg.position[0]:.3f}, "
                f"Tilt: {msg.position[1]:.3f}, "
                f"Zoom: {msg.position[2]:.3f}"
            )
    
    def status_string_callback(self, msg):
        """Callback for status string messages."""
        self.last_status_string = msg.data
        self.get_logger().debug(f"Status String: {msg.data}")
    
    def ptz_cmd_callback(self, msg):
        """Callback for PTZ command messages."""
        self.last_ptz_cmd = msg
        
        # Log PTZ commands
        self.get_logger().debug(
            f"PTZ Command - Pan: {msg.angular.z:.3f}, "
            f"Tilt: {msg.linear.y:.3f}, "
            f"Zoom: {msg.linear.z:.3f}"
        )
    
    def display_status(self):
        """Display current PTZ status."""
        current_time = datetime.now()
        
        # Clear screen and move cursor to top
        print("\033[H\033[J", end="")
        
        # Print header
        print("="*80)
        print(f"PTZ STATUS MONITOR - {current_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        # Joint State Status
        print("\n📊 JOINT STATE STATUS:")
        if self.last_joint_state:
            if len(self.last_joint_state.name) >= 3:
                print(f"  Joint Names: {', '.join(self.last_joint_state.name[:3])}")
            
            if len(self.last_joint_state.position) >= 3:
                print(f"  Positions:   Pan={self.last_joint_state.position[0]:+7.3f} | "
                      f"Tilt={self.last_joint_state.position[1]:+7.3f} | "
                      f"Zoom={self.last_joint_state.position[2]:+7.3f}")
            
            if len(self.last_joint_state.velocity) >= 3:
                print(f"  Velocities:  Pan={self.last_joint_state.velocity[0]:+7.3f} | "
                      f"Tilt={self.last_joint_state.velocity[1]:+7.3f} | "
                      f"Zoom={self.last_joint_state.velocity[2]:+7.3f}")
            
            # Time since last update
            if self.last_update_time:
                time_diff = (current_time - self.last_update_time).total_seconds()
                print(f"  Last Update: {time_diff:.1f} seconds ago")
        else:
            print("  ❌ No joint state messages received")
        
        # Status String
        print("\n📝 STATUS STRING:")
        if self.last_status_string:
            print(f"  {self.last_status_string}")
        else:
            print("  ❌ No status string messages received")
        
        # PTZ Commands
        print("\n🎮 LAST PTZ COMMAND:")
        if self.last_ptz_cmd:
            print(f"  Pan Speed:   {self.last_ptz_cmd.angular.z:+7.3f}")
            print(f"  Tilt Speed:  {self.last_ptz_cmd.linear.y:+7.3f}")
            print(f"  Zoom Speed:  {self.last_ptz_cmd.linear.z:+7.3f}")
            
            # Check if camera is moving
            is_moving = (abs(self.last_ptz_cmd.angular.z) > 0.01 or 
                        abs(self.last_ptz_cmd.linear.y) > 0.01 or 
                        abs(self.last_ptz_cmd.linear.z) > 0.01)
            
            status_icon = "🔄" if is_moving else "⏸️"
            status_text = "MOVING" if is_moving else "STOPPED"
            print(f"  Status:      {status_icon} {status_text}")
        else:
            print("  ❌ No PTZ command messages received")
        
        # Connection Status
        print("\n🔗 CONNECTION STATUS:")
        topics_info = [
            ("Joint State", self.last_joint_state is not None),
            ("Status String", self.last_status_string is not None),
            ("PTZ Commands", self.last_ptz_cmd is not None),
        ]
        
        for topic_name, is_connected in topics_info:
            icon = "✅" if is_connected else "❌"
            print(f"  {icon} {topic_name}")
        
        # Instructions
        print("\n" + "="*80)
        print("Press Ctrl+C to exit")
        print("="*80)
        
        # Flush output
        import sys
        sys.stdout.flush()


def main():
    """Main function."""
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # Create monitor
        monitor = PTZStatusMonitor()
        
        # Spin the node
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\nPTZ Status Monitor interrupted by user")
    except Exception as e:
        print(f"Error in PTZ Status Monitor: {str(e)}")
    finally:
        # Cleanup
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 