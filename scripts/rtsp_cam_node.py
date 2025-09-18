#!/usr/bin/env python3
"""
Generic RTSP Camera Node for ROS 1 Noetic

This script runs the main camera streaming node for generic RTSP cameras.
It supports both main and secondary streams with H.264/H.265 encoding and multiple decoders.
Optimized for Jetson Orin NX with hardware acceleration.
"""

import sys
import argparse
import rospy

from rtsp_cam.camera_streamer import RTSPCameraNode


def main():
    """Main function for the camera node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='RTSP Camera Node for ROS 1 Noetic')
    parser.add_argument('--sub-stream', action='store_true', 
                       help='Run as secondary stream node')
    parser.add_argument('--log-level', default='info', 
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    args = parser.parse_args()
    
    # Determine stream type
    stream_type = "sub" if args.sub_stream else "main"
    
    try:
        # Create camera node
        camera_node = RTSPCameraNode(stream_type)
        
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