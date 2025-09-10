# PTZ Test Scripts for Jidetech Camera

This directory contains test scripts to verify and debug the PTZ (Pan-Tilt-Zoom) functionality of the Jidetech camera package.

## Test Scripts

### 1. `test_camera_interface.py` - Camera Interface Discovery

**🚨 START HERE FIRST!** This script discovers which communication method works with your camera.

**Usage:**
```bash
# Test all interfaces (recommended)
python3 test_camera_interface.py --ip 192.168.0.18 --username admin --password admin

# Test specific interfaces
python3 test_camera_interface.py --test connectivity  # Network connectivity
python3 test_camera_interface.py --test http         # HTTP endpoints
python3 test_camera_interface.py --test onvif        # ONVIF protocol
python3 test_camera_interface.py --test cgi          # CGI PTZ commands
python3 test_camera_interface.py --test info         # Camera info discovery
```

**What it does:**
- Tests network connectivity and open ports
- Discovers working HTTP endpoints
- Tests ONVIF protocol compatibility
- Tries various PTZ command formats
- Identifies camera model and capabilities

### 2. `test_ptz_commands.py` - Comprehensive PTZ Test Suite

This script runs a comprehensive test suite to verify all PTZ functions.

**Usage:**
```bash
# Run all tests
python3 test_ptz_commands.py --test all

# Run specific test types
python3 test_ptz_commands.py --test basic      # Basic movements only
python3 test_ptz_commands.py --test combined   # Combined movements
python3 test_ptz_commands.py --test speed      # Speed variations
python3 test_ptz_commands.py --test circular   # Circular movement pattern
python3 test_ptz_commands.py --test preset     # Preset positions
python3 test_ptz_commands.py --test stress     # Stress test

# Send manual PTZ commands
python3 test_ptz_commands.py --pan 0.5 --tilt 0.3 --zoom 0.2 --duration 3.0
```

**Test Types:**
- **Basic**: Tests individual pan, tilt, and zoom movements
- **Combined**: Tests simultaneous movements (pan+tilt, pan+zoom, etc.)
- **Speed**: Tests different movement speeds
- **Circular**: Tests smooth circular movement patterns
- **Preset**: Tests movement to predefined positions
- **Stress**: Random rapid movements for stress testing

### 2. `manual_ptz_control.py` - Interactive PTZ Control

This script provides keyboard-based manual control of the PTZ camera.

**Usage:**
```bash
python3 manual_ptz_control.py
```

**Controls:**
- **Pan**: `A`/`D` or Left/Right arrows
- **Tilt**: `W`/`S` or Up/Down arrows  
- **Zoom**: `Q`/`E` or `+`/`-`
- **Speed**: `1`-`9` (1=slow, 9=fast)
- **Stop**: `0` or `Space`
- **Help**: `H`
- **Exit**: `ESC` or `Ctrl+C`

### 3. `ptz_status_monitor.py` - PTZ Status Monitor

This script monitors PTZ status messages in real-time.

**Usage:**
```bash
python3 ptz_status_monitor.py
```

**Features:**
- Real-time PTZ position and velocity display
- PTZ command monitoring
- Connection status indicators
- Live status updates

## Prerequisites

Before running the tests, ensure:

1. **ROS 2 is running:**
   ```bash
   # Source ROS 2
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Jidetech camera package is launched:**
   ```bash
   ros2 launch jidetech_camera jidetech_camera.launch.py
   ```

3. **Required Python packages are installed:**
   ```bash
   sudo apt install python3-rclpy python3-geometry-msgs python3-sensor-msgs python3-std-msgs
   ```

## Testing Workflow

### Step 1: Launch the Camera System
```bash
# Terminal 1: Launch the camera system
ros2 launch jidetech_camera jidetech_camera.launch.py
```

### Step 2: Monitor PTZ Status
```bash
# Terminal 2: Monitor PTZ status
cd ~/ros2_ws/src/jidetech_camera/test
python3 ptz_status_monitor.py
```

### Step 3: Test PTZ Commands
```bash
# Terminal 3: Run PTZ tests
cd ~/ros2_ws/src/jidetech_camera/test

# Option A: Run comprehensive test suite
python3 test_ptz_commands.py --test all

# Option B: Manual interactive control
python3 manual_ptz_control.py

# Option C: Send specific commands
python3 test_ptz_commands.py --pan 0.5 --duration 2.0
```

## Troubleshooting

### Common Issues

1. **No PTZ response:**
   - Check if PTZ controller node is running: `ros2 node list`
   - Verify PTZ is enabled in config: `ptz_enabled: true`
   - Check camera IP and credentials in config file

2. **Connection timeout:**
   - Verify camera IP address is reachable: `ping 192.168.0.18`
   - Check ONVIF port (8999) or HTTP port (80) accessibility
   - Try switching between ONVIF and HTTP mode: `use_onvif: false`

3. **Slow PTZ response:**
   - Adjust timeout in config: `timeout_ms: 10000`
   - Reduce PTZ speeds: `pan_speed: 0.3`, `tilt_speed: 0.3`

4. **PTZ moves in wrong direction:**
   - Check PTZ command mapping in `ptz_controller.py`
   - Verify camera's PTZ coordinate system

### Debug Commands

```bash
# Check running nodes
ros2 node list

# Check PTZ topics
ros2 topic list | grep ptz

# Monitor PTZ commands
ros2 topic echo /camera/ptz_cmd

# Monitor PTZ status
ros2 topic echo /camera/ptz_status

# Check PTZ controller parameters
ros2 param list /jidetech_ptz_controller
```

## PTZ Command Format

The PTZ commands use the `geometry_msgs/Twist` message format:

```python
# PTZ Command Mapping
msg.angular.z = pan     # Pan: -1.0 (left) to +1.0 (right)
msg.linear.y = tilt     # Tilt: -1.0 (down) to +1.0 (up)  
msg.linear.z = zoom     # Zoom: -1.0 (out) to +1.0 (in)
```

## Expected Test Results

When the PTZ system is working correctly:

1. **Status Monitor** should show:
   - ✅ Joint State messages received
   - ✅ Status String messages received  
   - ✅ PTZ Commands received
   - Real-time position/velocity updates

2. **Test Suite** should show:
   - All basic movements execute without errors
   - Camera responds to commands within 1-2 seconds
   - Smooth transitions between movements
   - No timeout or connection errors

3. **Manual Control** should provide:
   - Responsive camera movement
   - Accurate directional control
   - Variable speed control
   - Immediate stop functionality

If any tests fail, check the troubleshooting section and verify your camera configuration. 