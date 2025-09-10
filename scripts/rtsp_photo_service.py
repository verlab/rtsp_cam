#!/usr/bin/env python3
"""
Jidetech Photo Service Node

This script runs the photo capture service node for Jidetech IP cameras.
It provides a ROS service to capture photos from the camera.
"""

import sys
import argparse
import rclpy
from rclpy.executors import MultiThreadedExecutor

from jidetech_camera.photo_service import JidetechPhotoService


def main(args=None):
    """Main function for the photo service node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Jidetech Photo Service Node')
    parser.add_argument('--ros-args', nargs='*', help='ROS arguments')
    parser.add_argument('--log-level', default='info', 
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    # Parse known args (ROS 2 will handle unknown args)
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS 2
    rclpy.init(args=unknown)
    
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