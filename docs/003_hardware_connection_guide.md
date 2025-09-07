# Hardware Connection Issues Guide

## Common Hardware Errors

### 1. Serial Port Error (`/dev/ttyUSB0`)
**Error:**
```
[ERROR] [robotis_controller]: PORT [/dev/ttyUSB0] SETUP ERROR! (baudrate: 2000000)
[PortHandlerLinux::SetupPort] Error opening serial port!
```

**Causes:**
- Robot hardware not connected
- USB cable disconnected
- Serial device permissions
- Wrong device path

**Solutions:**

#### Check USB Connection
```bash
# List available USB devices
lsusb

# Check for ttyUSB devices
ls -la /dev/ttyUSB*

# Check device permissions
ls -la /dev/ttyUSB0
```

#### Fix Permissions
```bash
# Add user to dialout group (required for serial access)
sudo usermod -a -G dialout $USER

# Apply group changes (logout/login or run):
newgrp dialout

# Set device permissions temporarily
sudo chmod 666 /dev/ttyUSB0
```

#### Alternative Device Paths
If `/dev/ttyUSB0` doesn't exist, try:
- `/dev/ttyUSB1`, `/dev/ttyUSB2`
- `/dev/ttyACM0`, `/dev/ttyACM1`
- Check dmesg for device detection: `dmesg | grep tty`

### 2. Camera Connection Error
**Error:**
```
filesystem error: directory iterator cannot open directory: No such file or directory [/sys/class/video4linux/]
```

**Causes:**
- No USB camera connected
- Camera driver not loaded
- Video4Linux not available

**Solutions:**

#### Check Video Devices
```bash
# List video devices
ls -la /dev/video*

# Check video4linux directory
ls -la /sys/class/video4linux/

# List USB video devices
lsusb | grep -i video
```

#### Test Camera Access
```bash
# Test camera with v4l2-utils
v4l2-ctl --list-devices

# Test with simple capture
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test.jpg
```

## Running Without Hardware

### 1. Disable Hardware Nodes
Create custom launch file without hardware dependencies:
```python
# custom_op3_sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Only launch software components
        Node(
            package='op3_manager',
            executable='op3_manager',
            name='op3_manager',
            parameters=[
                {'use_dummy_device': True}  # If available
            ]
        )
    ])
```

### 2. Mock Hardware Mode
Some packages support simulation mode:
```bash
# Check for dummy/simulation parameters
ros2 param list /op3_manager

# Set simulation mode if available
ros2 param set /op3_manager use_dummy_device true
```

### 3. Test Individual Components
```bash
# Test framework without hardware
ros2 run robotis_controller robotis_controller

# Test specific modules
ros2 run op3_walking_module op3_walking_module
```

## Hardware Requirements

### ROBOTIS OP3 Hardware
- **Main Controller**: Intel NUC or equivalent
- **Sub Controller**: OpenCR board
- **Motors**: 20x Dynamixel XM430-W350
- **Sensors**: IMU, camera
- **Communication**: USB to Serial (U2D2 or similar)

### Connection Checklist
- [ ] Power supply connected (12V for motors, 19V for NUC)
- [ ] USB cable from NUC to OpenCR (should appear as /dev/ttyUSB0)
- [ ] All motor connections secure
- [ ] Camera USB connection
- [ ] Power switch ON
- [ ] Status LEDs active on OpenCR

### Serial Configuration
- **Baudrate**: 2,000,000 bps
- **Protocol**: Dynamixel Protocol 2.0
- **Device**: `/dev/ttyUSB0` (default)
- **IDs**: Motors (1-20), OpenCR (200)

## Troubleshooting Commands

```bash
# Check system resources
dmesg | tail -20

# Monitor USB connections
sudo udevadm monitor --environment --udev

# Test serial communication
sudo minicom -D /dev/ttyUSB0 -b 2000000

# Check ROS 2 environment
ros2 doctor
```