#!/usr/bin/env python3
"""
Setup script for Jidetech Camera ROS 2 Package
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rtsp_cam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rezeck',
    maintainer_email='rezeck@ufmg.br',
    description='Generic ROS 2 package for RTSP cameras with streaming, PTZ control, and photo capture capabilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_cam_node = rtsp_cam.camera_streamer:main',
            'rtsp_ptz_controller = rtsp_cam.ptz_controller:main',
            'rtsp_photo_service = rtsp_cam.photo_service:main',
        ],
    },
) 