#!/usr/bin/env python3
"""
PTZ Command Test Script for Jidetech Camera

This script tests various PTZ (Pan-Tilt-Zoom) commands to verify the functionality
of the Jidetech camera PTZ controller.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import argparse
import sys


class PTZTester(Node):
    """Test node for PTZ commands."""
    
    def __init__(self):
        super().__init__('ptz_tester')
        
        # Create PTZ command publisher
        self.ptz_pub = self.create_publisher(Twist, '/camera/ptz_cmd', 10)
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        
        self.get_logger().info("PTZ Tester initialized")
    
    def send_ptz_command(self, pan=0.0, tilt=0.0, zoom=0.0, duration=2.0):
        """
        Send PTZ command.
        
        Args:
            pan: Pan speed (-1.0 to 1.0)
            tilt: Tilt speed (-1.0 to 1.0) 
            zoom: Zoom speed (-1.0 to 1.0)
            duration: Duration to hold the command (seconds)
        """
        msg = Twist()
        msg.angular.z = pan    # Pan
        msg.linear.y = tilt    # Tilt
        msg.linear.z = zoom    # Zoom
        
        self.get_logger().info(f"Sending PTZ command: pan={pan:.2f}, tilt={tilt:.2f}, zoom={zoom:.2f}")
        
        # Send command for specified duration
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.ptz_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop movement
        self.stop_ptz()
    
    def stop_ptz(self):
        """Stop all PTZ movement."""
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        self.ptz_pub.publish(msg)
        self.get_logger().info("PTZ stopped")
        time.sleep(0.5)
    
    def test_basic_movements(self):
        """Test basic PTZ movements."""
        self.get_logger().info("=== Testing Basic PTZ Movements ===")
        
        # Test pan left
        self.get_logger().info("Testing Pan Left...")
        self.send_ptz_command(pan=-0.5, duration=2.0)
        time.sleep(1.0)
        
        # Test pan right
        self.get_logger().info("Testing Pan Right...")
        self.send_ptz_command(pan=0.5, duration=2.0)
        time.sleep(1.0)
        
        # Test tilt up
        self.get_logger().info("Testing Tilt Up...")
        self.send_ptz_command(tilt=0.5, duration=2.0)
        time.sleep(1.0)
        
        # Test tilt down
        self.get_logger().info("Testing Tilt Down...")
        self.send_ptz_command(tilt=-0.5, duration=2.0)
        time.sleep(1.0)
        
        # Test zoom in
        self.get_logger().info("Testing Zoom In...")
        self.send_ptz_command(zoom=0.5, duration=2.0)
        time.sleep(1.0)
        
        # Test zoom out
        self.get_logger().info("Testing Zoom Out...")
        self.send_ptz_command(zoom=-0.5, duration=2.0)
        time.sleep(1.0)
    
    def test_combined_movements(self):
        """Test combined PTZ movements."""
        self.get_logger().info("=== Testing Combined PTZ Movements ===")
        
        # Pan left + tilt up
        self.get_logger().info("Testing Pan Left + Tilt Up...")
        self.send_ptz_command(pan=-0.3, tilt=0.3, duration=2.0)
        time.sleep(1.0)
        
        # Pan right + tilt down
        self.get_logger().info("Testing Pan Right + Tilt Down...")
        self.send_ptz_command(pan=0.3, tilt=-0.3, duration=2.0)
        time.sleep(1.0)
        
        # Pan + zoom
        self.get_logger().info("Testing Pan + Zoom...")
        self.send_ptz_command(pan=0.2, zoom=0.3, duration=2.0)
        time.sleep(1.0)
        
        # All three movements
        self.get_logger().info("Testing Pan + Tilt + Zoom...")
        self.send_ptz_command(pan=0.2, tilt=0.2, zoom=0.2, duration=2.0)
        time.sleep(1.0)
    
    def test_speed_variations(self):
        """Test different speed variations."""
        self.get_logger().info("=== Testing Speed Variations ===")
        
        speeds = [0.1, 0.3, 0.5, 0.7, 1.0]
        
        for speed in speeds:
            self.get_logger().info(f"Testing Pan at speed {speed}...")
            self.send_ptz_command(pan=speed, duration=1.5)
            time.sleep(0.5)
            
            self.send_ptz_command(pan=-speed, duration=1.5)
            time.sleep(0.5)
    
    def test_circular_movement(self):
        """Test circular movement pattern."""
        self.get_logger().info("=== Testing Circular Movement ===")
        
        import math
        
        # Create circular movement
        steps = 16
        radius = 0.5
        duration_per_step = 0.5
        
        for i in range(steps):
            angle = 2 * math.pi * i / steps
            pan = radius * math.cos(angle)
            tilt = radius * math.sin(angle)
            
            self.get_logger().info(f"Circular step {i+1}/{steps}: pan={pan:.2f}, tilt={tilt:.2f}")
            self.send_ptz_command(pan=pan, tilt=tilt, duration=duration_per_step)
    
    def test_preset_positions(self):
        """Test preset positions."""
        self.get_logger().info("=== Testing Preset Positions ===")
        
        positions = [
            ("Center", 0.0, 0.0, 0.0),
            ("Top-Left", -0.5, 0.5, 0.0),
            ("Top-Right", 0.5, 0.5, 0.0),
            ("Bottom-Left", -0.5, -0.5, 0.0),
            ("Bottom-Right", 0.5, -0.5, 0.0),
            ("Zoom In Center", 0.0, 0.0, 0.7),
            ("Zoom Out Center", 0.0, 0.0, -0.7),
        ]
        
        for name, pan, tilt, zoom in positions:
            self.get_logger().info(f"Moving to {name}...")
            self.send_ptz_command(pan=pan, tilt=tilt, zoom=zoom, duration=2.0)
            time.sleep(1.0)
    
    def test_stress_test(self):
        """Stress test with rapid movements."""
        self.get_logger().info("=== PTZ Stress Test ===")
        
        import random
        
        for i in range(10):
            pan = random.uniform(-1.0, 1.0)
            tilt = random.uniform(-1.0, 1.0)
            zoom = random.uniform(-1.0, 1.0)
            
            self.get_logger().info(f"Stress test {i+1}/10: pan={pan:.2f}, tilt={tilt:.2f}, zoom={zoom:.2f}")
            self.send_ptz_command(pan=pan, tilt=tilt, zoom=zoom, duration=1.0)
            time.sleep(0.2)
    
    def run_all_tests(self):
        """Run all PTZ tests."""
        self.get_logger().info("Starting comprehensive PTZ test suite...")
        
        try:
            self.test_basic_movements()
            time.sleep(2.0)
            
            self.test_combined_movements()
            time.sleep(2.0)
            
            self.test_speed_variations()
            time.sleep(2.0)
            
            self.test_circular_movement()
            time.sleep(2.0)
            
            self.test_preset_positions()
            time.sleep(2.0)
            
            self.test_stress_test()
            
            # Return to center
            self.get_logger().info("Returning to center position...")
            self.send_ptz_command(pan=0.0, tilt=0.0, zoom=0.0, duration=2.0)
            
            self.get_logger().info("=== All PTZ tests completed successfully! ===")
            
        except Exception as e:
            self.get_logger().error(f"PTZ test failed: {str(e)}")
            self.stop_ptz()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='PTZ Command Tester')
    parser.add_argument('--test', choices=['basic', 'combined', 'speed', 'circular', 'preset', 'stress', 'all'], 
                       default='all', help='Test type to run')
    parser.add_argument('--pan', type=float, help='Manual pan command (-1.0 to 1.0)')
    parser.add_argument('--tilt', type=float, help='Manual tilt command (-1.0 to 1.0)')
    parser.add_argument('--zoom', type=float, help='Manual zoom command (-1.0 to 1.0)')
    parser.add_argument('--duration', type=float, default=2.0, help='Command duration in seconds')
    
    args = parser.parse_args()
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # Create tester node
        tester = PTZTester()
        
        # Manual command mode
        if args.pan is not None or args.tilt is not None or args.zoom is not None:
            pan = args.pan if args.pan is not None else 0.0
            tilt = args.tilt if args.tilt is not None else 0.0
            zoom = args.zoom if args.zoom is not None else 0.0
            
            tester.get_logger().info(f"Sending manual PTZ command: pan={pan}, tilt={tilt}, zoom={zoom}")
            tester.send_ptz_command(pan, tilt, zoom, args.duration)
        
        # Test mode
        elif args.test == 'basic':
            tester.test_basic_movements()
        elif args.test == 'combined':
            tester.test_combined_movements()
        elif args.test == 'speed':
            tester.test_speed_variations()
        elif args.test == 'circular':
            tester.test_circular_movement()
        elif args.test == 'preset':
            tester.test_preset_positions()
        elif args.test == 'stress':
            tester.test_stress_test()
        elif args.test == 'all':
            tester.run_all_tests()
        
    except KeyboardInterrupt:
        print("\nPTZ test interrupted by user")
    except Exception as e:
        print(f"Error in PTZ test: {str(e)}")
    finally:
        # Cleanup
        if 'tester' in locals():
            tester.stop_ptz()
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 