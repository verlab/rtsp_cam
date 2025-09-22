#!/usr/bin/env python3
"""
RTSP Photo Service Node for ROS 1 Noetic

This script runs the photo capture service node for RTSP cameras.
It provides a ROS service to capture photos from the camera.
Optimized for Jetson Orin NX.
"""

import sys
import argparse
import rospy

from rtsp_cam.photo_service import RTSPPhotoService


def main():
    """Main function for the photo service node."""
    # Parse remaining arguments (filter out ROS args)
    non_ros_args = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(description='RTSP Photo Service Node for ROS 1 Noetic')
    parser.add_argument('--log-level', default='info', 
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    args = parser.parse_args(non_ros_args[1:])
    
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