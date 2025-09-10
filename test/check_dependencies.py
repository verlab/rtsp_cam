#!/usr/bin/env python3
"""
Dependency Checker for Jidetech Camera Tests

This script checks if all required dependencies are available for running the camera tests.
"""

import sys
import subprocess
import importlib
import platform


def check_python_version():
    """Check Python version."""
    print("🐍 Checking Python version...")
    version = sys.version_info
    if version.major >= 3 and version.minor >= 8:
        print(f"✅ Python {version.major}.{version.minor}.{version.micro} - OK")
        return True
    else:
        print(f"❌ Python {version.major}.{version.minor}.{version.micro} - Need Python 3.8+")
        return False


def check_python_packages():
    """Check required Python packages."""
    print("\n📦 Checking Python packages...")
    
    required_packages = [
        'requests',
        'xml.etree.ElementTree',
        'socket',
        'json',
        'time',
        'argparse',
        'subprocess',
        'threading',
        'termios',  # For manual PTZ control
        'tty',     # For manual PTZ control
    ]
    
    optional_packages = [
        'rclpy',           # For ROS 2 tests
        'geometry_msgs',   # For ROS 2 tests
        'sensor_msgs',     # For ROS 2 tests
        'std_msgs',        # For ROS 2 tests
    ]
    
    all_good = True
    
    # Check required packages
    for package in required_packages:
        try:
            importlib.import_module(package)
            print(f"✅ {package} - Available")
        except ImportError:
            print(f"❌ {package} - Missing")
            all_good = False
    
    # Check optional packages (for ROS 2)
    print("\n📦 Checking optional ROS 2 packages...")
    ros2_available = True
    
    for package in optional_packages:
        try:
            importlib.import_module(package)
            print(f"✅ {package} - Available")
        except ImportError:
            print(f"⚠️  {package} - Missing (ROS 2 tests won't work)")
            ros2_available = False
    
    if ros2_available:
        print("✅ ROS 2 packages available - Full functionality")
    else:
        print("⚠️  ROS 2 packages missing - Camera interface tests will work, PTZ tests won't")
    
    return all_good


def check_system_commands():
    """Check system commands."""
    print("\n🔧 Checking system commands...")
    
    commands = {
        'ping': 'Network connectivity testing',
        'curl': 'HTTP testing (optional)',
        'wget': 'HTTP testing (optional)',
    }
    
    for cmd, description in commands.items():
        try:
            result = subprocess.run([cmd, '--help'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=3)
            if result.returncode == 0 or 'usage' in result.stderr.lower():
                print(f"✅ {cmd} - Available ({description})")
            else:
                print(f"⚠️  {cmd} - Not working properly ({description})")
        except FileNotFoundError:
            if cmd == 'ping':
                print(f"⚠️  {cmd} - Not found (will use socket fallback)")
            else:
                print(f"⚠️  {cmd} - Not found ({description})")
        except subprocess.TimeoutExpired:
            print(f"⚠️  {cmd} - Timeout ({description})")
        except Exception as e:
            print(f"❌ {cmd} - Error: {e}")


def check_network_tools():
    """Check network diagnostic tools."""
    print("\n🌐 Checking network tools...")
    
    # Test if we can create sockets
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.close()
        print("✅ Socket creation - OK")
    except Exception as e:
        print(f"❌ Socket creation failed: {e}")
    
    # Test if we can make HTTP requests
    try:
        import requests
        # Test with a quick request to a reliable server
        response = requests.get('http://httpbin.org/status/200', timeout=3)
        if response.status_code == 200:
            print("✅ HTTP requests - OK")
        else:
            print("⚠️  HTTP requests - Unexpected response")
    except requests.exceptions.Timeout:
        print("⚠️  HTTP requests - Timeout (may be network issue)")
    except requests.exceptions.ConnectionError:
        print("⚠️  HTTP requests - Connection error (may be network issue)")
    except Exception as e:
        print(f"❌ HTTP requests failed: {e}")


def install_missing_packages():
    """Provide installation instructions for missing packages."""
    print("\n📋 INSTALLATION INSTRUCTIONS")
    print("="*50)
    
    print("\n🐧 For Ubuntu/Debian:")
    print("sudo apt update")
    print("sudo apt install -y python3-pip python3-requests python3-setuptools")
    print("sudo apt install -y iputils-ping curl wget")  # ping and network tools
    print("sudo apt install -y python3-termios")  # For manual control
    
    print("\n🍎 For macOS:")
    print("# Install Homebrew if not already installed")
    print("# /bin/bash -c \"$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\"")
    print("brew install python3")
    print("pip3 install requests")
    
    print("\n🪟 For Windows:")
    print("# Install Python from python.org")
    print("pip install requests")
    print("# Ping is built-in on Windows")
    
    print("\n🤖 For ROS 2 (if needed):")
    print("sudo apt install -y ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs")
    print("pip3 install rclpy")


def main():
    """Main dependency check."""
    print("🔍 JIDETECH CAMERA TEST DEPENDENCY CHECKER")
    print("="*50)
    
    checks = [
        check_python_version(),
        check_python_packages(),
    ]
    
    # Always run these (they provide useful info even if some fail)
    check_system_commands()
    check_network_tools()
    
    print("\n" + "="*50)
    print("📋 SUMMARY")
    print("="*50)
    
    if all(checks):
        print("✅ All required dependencies are available!")
        print("🚀 You can run the camera interface tests:")
        print("   python3 test_camera_interface.py")
    else:
        print("❌ Some required dependencies are missing.")
        print("📋 See installation instructions below:")
        install_missing_packages()
    
    print(f"\n💻 System: {platform.system()} {platform.release()}")
    print(f"🐍 Python: {sys.version}")


if __name__ == '__main__':
    main() 