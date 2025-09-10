#!/usr/bin/env python3
"""
Generic RTSP Camera Node

This script runs the main camera streaming node for generic RTSP cameras.
It supports both main and secondary streams with H.264/H.265 encoding and multiple decoders.
"""

import sys
import argparse
import rclpy
from rclpy.executors import MultiThreadedExecutor

from rtsp_cam.camera_streamer import RTSPCameraNode


def main(args=None):
    """Main function for the camera node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='RTSP Camera Node')
    parser.add_argument('--sub-stream', action='store_true', 
                       help='Run as secondary stream node')
    parser.add_argument('--ros-args', nargs='*', help='ROS arguments')
    parser.add_argument('--log-level', default='info', 
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    # Parse known args (ROS 2 will handle unknown args)
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS 2
    rclpy.init(args=unknown)
    
    # Determine stream type
    stream_type = "sub" if args.sub_stream else "main"
    
    try:
        # Create camera node
        camera_node = RTSPCameraNode(stream_type)
        
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