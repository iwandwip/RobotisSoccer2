# CycloneDDS Configuration Fix

## Problem
ROS 2 nodes fail to start with error:
```
can't open configuration file file:///home/robotis/.ros/cyclonedds.xml
rmw_create_node: failed to create domain, error Error
failed to initialize rcl node
```

## Root Cause
The `CYCLONEDDS_URI` environment variable points to `/home/robotis/.ros/cyclonedds.xml` but the `robotis` user doesn't exist on this system.

## Solution

### 1. Identify Current Configuration
```bash
# Check current setting
echo $CYCLONEDDS_URI

# Find where it's defined
grep -r "CYCLONEDDS_URI" ~/.bashrc ~/.zshrc
```

### 2. Update Configuration
Edit `~/.bashrc`:
```bash
# Old path (robotis user doesn't exist on this system):
# export CYCLONEDDS_URI=file:///home/robotis/.ros/cyclonedds.xml
# Corrected path for current user:
export CYCLONEDDS_URI=file:///home/iwandwp/.ros/cyclonedds.xml
```

### 3. Apply Changes
```bash
# Reload configuration
source ~/.bashrc

# Or for immediate effect in current session:
export CYCLONEDDS_URI=file:///home/iwandwp/.ros/cyclonedds.xml
```

### 4. Verify Fix
```bash
# Test the launch
cd ~/robotis_ws
ros2 launch op3_bringup op3_bringup.launch.py
```

## Expected Result
Nodes should start successfully without CycloneDDS configuration errors. Hardware-related errors may still appear if robot is not connected.

## CycloneDDS Configuration Details
The working configuration file is at `/home/iwandwp/.ros/cyclonedds.xml`:
- Uses loopback interface (127.0.0.1)
- Disables multicast
- Sets appropriate buffer sizes for robot communication
- Configured for ROS_DOMAIN_ID=1