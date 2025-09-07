# ROBOTIS OP3 Webots Simulation Setup Guide for WSL2

## Overview
Complete guide for setting up ROBOTIS OP3 Webots simulation on WSL2 for development and testing without physical hardware.

## Prerequisites

### System Requirements
- WSL2 with Ubuntu 22.04/24.04
- ROS2 Jazzy installed and configured
- X11 forwarding configured for GUI applications
- ROBOTIS OP3 workspace built with jazzy-devel branches

### Required Packages Installation
```bash
# Update package list
sudo apt update

# Install Webots ROS2 package
sudo apt install ros-jazzy-webots-ros2

# Install additional Webots dependencies
sudo apt install ros-jazzy-webots-ros2-driver
sudo apt install ros-jazzy-webots-ros2-msgs

# Verify installation
ros2 pkg list | grep webots
```

## Available OP3 Packages (Verified)
After building ROBOTIS OP3 workspace, the following packages are available:

### Core Packages
- `op3_bringup` - Main robot bringup and launch files
- `op3_manager` - Robot manager with simulation mode support
- `op3_description` - Robot URDF and visual models
- `op3_gui_demo` - GUI demo interface

### Module Packages  
- `op3_action_module` - Pre-defined action sequences
- `op3_walking_module` - Walking pattern generation
- `op3_balance_control` - Dynamic balance control
- `op3_head_control_module` - Head movement control
- `op3_base_module` - Base functionality module

### Utility Packages
- `op3_action_editor` - Action sequence editor
- `op3_tuner_client` - Parameter tuning interface
- `op3_web_setting_tool` - Web-based configuration
- `op3_ball_detector` - Ball detection for soccer demo
- `op3_camera_setting_tool` - Camera configuration

## Available Launch Files (Verified)
```bash
# Core launch files
/op3_manager/launch/op3_simulation.launch.py     # Main simulation manager
/op3_manager/launch/op3_manager.launch.py        # Robot manager
/op3_gui_demo/launch/op3_demo.launch.py          # GUI demo interface
/op3_bringup/launch/op3_bringup.launch.py        # Robot bringup
/op3_bringup/launch/op3_bringup_visualization.launch.py  # RViz visualization
/op3_description/launch/op3_display.launch.py    # Model display

# Utility launch files
/op3_ball_detector/launch/ball_detector_from_usb_cam.launch.py
```

## Webots Simulation Setup

### Option 1: Full Webots Simulation (Recommended)
```bash
# Terminal 1: Launch Webots simulation
ros2 launch op3_webots_ros2 robot_launch.py

# Terminal 2: Launch OP3 simulation manager
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 3: Launch GUI demo
ros2 launch op3_gui_demo op3_demo.launch.py

# Terminal 4 (Optional): Launch additional RViz
rviz2
```

### Option 2: Webots with Custom Configuration
```bash
# Launch Webots with specific world file
ros2 launch op3_webots_ros2 robot_launch.py world:=op3_world.wbt

# Launch with custom parameters
ros2 launch op3_webots_ros2 robot_launch.py use_sim_time:=true
```

## Alternative: RViz-Only Simulation

### For Development Without Physics Engine
```bash
# Terminal 1: Launch simulation manager (critical for GUI demo)
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 2: Launch RViz visualization
ros2 launch op3_bringup op3_bringup_visualization.launch.py

# Terminal 3: Launch GUI demo
ros2 launch op3_gui_demo op3_demo.launch.py
```

This setup provides:
- Robot model visualization in RViz
- Joint state publishing and visualization
- GUI demo functionality (Init Pose, Walking, Actions)
- Real-time joint movement feedback

## WSL2 Specific Configuration

### X11 Forwarding Setup
```bash
# Add to ~/.bashrc
export DISPLAY=:0.0
export LIBGL_ALWAYS_INDIRECT=1

# Test GUI functionality
xclock

# For Webots specifically
export WEBOTS_HOME=/usr/local/webots
```

### Performance Optimization
```bash
# Increase WSL2 memory limit in .wslconfig
[wsl2]
memory=8GB
processors=4
```

## Simulation Workflow

### Basic Testing Sequence
1. **Start Simulation Manager**: Handles robot control and state management
2. **Start Visualization**: Either Webots or RViz for visual feedback
3. **Launch GUI Demo**: Provides user interface for robot control
4. **Test Functions**: Init Pose, Walking, Head Control, Actions

### Debugging Commands
```bash
# Check running nodes
ros2 node list

# Check available topics
ros2 topic list

# Check services (should include robotis services)
ros2 service list | grep robotis

# Monitor joint states
ros2 topic echo /joint_states

# Test service calls
ros2 service call /robotis/action_module/set_joint_states_msg std_msgs/String "data: 'ini_pose'"
```

## Expected Behavior

### When GUI Demo "Init Pose" is Clicked
1. GUI Demo sends command to `/robotis/action_module/set_joint_states_msg`
2. OP3 Manager receives and processes the command
3. Joint states are published to `/joint_states` topic
4. Webots/RViz updates robot visualization showing movement to initial pose
5. Console shows: `[op3_gui_demo]: Go to robot initial pose.`

### Successful Setup Indicators
- ✅ Robot model appears in Webots/RViz
- ✅ GUI Demo window opens without errors
- ✅ Init Pose button causes visible robot movement
- ✅ Joint states update in real-time
- ✅ Console shows successful command processing

## Troubleshooting

### Common Issues

#### Webots Not Starting
```bash
# Check if Webots is properly installed
which webots

# Verify ROS2 Webots package
ros2 pkg list | grep webots

# Check X11 forwarding
echo $DISPLAY
```

#### GUI Demo Not Responding
```bash
# Ensure simulation manager is running first
ros2 node list | grep op3_manager

# Check if robotis services are available
ros2 service list | grep robotis
```

#### No Robot Visualization
```bash
# Check robot description
ros2 param get /robot_state_publisher robot_description

# Verify joint state publisher
ros2 topic list | grep joint_states
```

## Additional Resources

### Webots Documentation
- [Webots ROS2 Integration](https://docs.ros.org/en/galactic/Tutorials/Advanced/Simulators/Webots.html)
- [Webots ROBOTIS OP3](https://cyberbotics.com/doc/guide/robotis-op3)

### ROBOTIS OP3 Resources
- [OP3 Simulation Manual](https://emanual.robotis.com/docs/en/platform/op3/simulation/)
- [OP3 Tutorials](https://emanual.robotis.com/docs/en/platform/op3/tutorials/)

---
*Guide created: September 7, 2025*
*Tested on: WSL2 Ubuntu with ROS2 Jazzy*
*ROBOTIS OP3 packages: jazzy-devel branch*