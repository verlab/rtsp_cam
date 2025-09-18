#!/usr/bin/env python3
"""
Generic RTSP Camera PTZ Controller Node for ROS 1 Noetic

This script runs the PTZ control node for RTSP cameras with PTZ capabilities.
It provides pan, tilt, and zoom control via HTTP interfaces.
Optimized for Jetson Orin NX.
"""

import sys
import argparse
import rospy

from rtsp_cam.ptz_controller import RTSPCameraPTZNode


def main():
    """Main function for the PTZ controller node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='RTSP Camera PTZ Controller Node for ROS 1 Noetic')
    parser.add_argument('--log-level', default='info', 
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    args = parser.parse_args()
    
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