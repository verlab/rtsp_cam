#!/usr/bin/env python3
"""
Manual PTZ Control Script for Jidetech Camera

This script provides keyboard-based manual control of PTZ functions.
Use arrow keys and other keys to control the camera.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time


class ManualPTZController(Node):
    """Manual PTZ controller with keyboard input."""
    
    def __init__(self):
        super().__init__('manual_ptz_controller')
        
        # Create PTZ command publisher
        self.ptz_pub = self.create_publisher(Twist, '/camera/ptz_cmd', 10)
        
        # Current PTZ state
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_zoom = 0.0
        
        # Control parameters
        self.step_size = 0.1
        self.max_speed = 1.0
        
        # Publisher timer
        self.timer = self.create_timer(0.1, self.publish_ptz_command)
        
        self.get_logger().info("Manual PTZ Controller initialized")
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions."""
        print("\n" + "="*50)
        print("    MANUAL PTZ CONTROL")
        print("="*50)
        print("Pan Control:")
        print("  A / Left Arrow  - Pan Left")
        print("  D / Right Arrow - Pan Right")
        print()
        print("Tilt Control:")
        print("  W / Up Arrow    - Tilt Up")
        print("  S / Down Arrow  - Tilt Down")
        print()
        print("Zoom Control:")
        print("  Q / +           - Zoom In")
        print("  E / -           - Zoom Out")
        print()
        print("Speed Control:")
        print("  1-9             - Set speed (1=slow, 9=fast)")
        print("  0               - Stop all movement")
        print()
        print("Other:")
        print("  SPACE           - Stop all movement")
        print("  H               - Show this help")
        print("  ESC / Ctrl+C    - Exit")
        print("="*50)
        print(f"Current step size: {self.step_size:.1f}")
        print("="*50)
    
    def publish_ptz_command(self):
        """Publish current PTZ command."""
        msg = Twist()
        msg.angular.z = self.current_pan
        msg.linear.y = self.current_tilt
        msg.linear.z = self.current_zoom
        
        self.ptz_pub.publish(msg)
    
    def set_pan(self, value):
        """Set pan value."""
        self.current_pan = max(-self.max_speed, min(self.max_speed, value))
        self.print_status()
    
    def set_tilt(self, value):
        """Set tilt value."""
        self.current_tilt = max(-self.max_speed, min(self.max_speed, value))
        self.print_status()
    
    def set_zoom(self, value):
        """Set zoom value."""
        self.current_zoom = max(-self.max_speed, min(self.max_speed, value))
        self.print_status()
    
    def stop_all(self):
        """Stop all movement."""
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_zoom = 0.0
        self.print_status()
    
    def set_step_size(self, size):
        """Set step size for movements."""
        self.step_size = max(0.1, min(1.0, size))
        print(f"Step size set to: {self.step_size:.1f}")
    
    def print_status(self):
        """Print current PTZ status."""
        print(f"\rPTZ Status - Pan: {self.current_pan:+.2f} | Tilt: {self.current_tilt:+.2f} | Zoom: {self.current_zoom:+.2f} | Step: {self.step_size:.1f}", end="", flush=True)
    
    def process_key(self, key):
        """Process keyboard input."""
        key_lower = key.lower()
        
        # Pan controls
        if key_lower == 'a' or key == '\x1b[D':  # A or Left arrow
            self.set_pan(-self.step_size)
        elif key_lower == 'd' or key == '\x1b[C':  # D or Right arrow
            self.set_pan(self.step_size)
        
        # Tilt controls
        elif key_lower == 'w' or key == '\x1b[A':  # W or Up arrow
            self.set_tilt(self.step_size)
        elif key_lower == 's' or key == '\x1b[B':  # S or Down arrow
            self.set_tilt(-self.step_size)
        
        # Zoom controls
        elif key_lower == 'q' or key == '+':  # Q or +
            self.set_zoom(self.step_size)
        elif key_lower == 'e' or key == '-':  # E or -
            self.set_zoom(-self.step_size)
        
        # Speed controls
        elif key.isdigit():
            if key == '0':
                self.stop_all()
            else:
                speed = int(key) * 0.1
                self.set_step_size(speed)
        
        # Stop
        elif key == ' ':  # Space
            self.stop_all()
        
        # Help
        elif key_lower == 'h':
            self.print_instructions()
        
        # Exit
        elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
            return False
        
        return True


def get_key():
    """Get a single keypress from stdin."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        
        # Handle arrow keys (escape sequences)
        if key == '\x1b':
            key += sys.stdin.read(2)
        
        return key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def keyboard_thread(controller):
    """Keyboard input thread."""
    try:
        while True:
            key = get_key()
            if not controller.process_key(key):
                break
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nKeyboard thread error: {e}")


def main():
    """Main function."""
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # Create controller
        controller = ManualPTZController()
        
        # Start keyboard input thread
        kb_thread = threading.Thread(target=keyboard_thread, args=(controller,), daemon=True)
        kb_thread.start()
        
        # Spin the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nManual PTZ control interrupted by user")
    except Exception as e:
        print(f"Error in manual PTZ control: {str(e)}")
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.stop_all()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 