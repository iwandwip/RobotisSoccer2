# ROBOTIS OP3 Working Simulation Commands

## Quick Reference Commands

### Option 1: Webots Full Simulation (Recommended for Complete Physics)

#### Terminal 1: Webots Simulation
```bash
# Start Webots physics simulation
ros2 launch op3_webots_ros2 robot_launch.py
```

#### Terminal 2: OP3 Simulation Manager  
```bash
# Start robot control manager (CRITICAL - must run before GUI demo)
ros2 launch op3_manager op3_simulation.launch.py
```

#### Terminal 3: GUI Demo Interface
```bash
# Start GUI control interface
ros2 launch op3_gui_demo op3_demo.launch.py
```

#### Terminal 4: Additional RViz (Optional)
```bash
# Additional visualization if needed
rviz2
```

### Option 2: RViz-Only Development Mode (Lightweight)

#### Terminal 1: Simulation Manager (REQUIRED)
```bash
# Robot controller - MUST run first for GUI demo to work
ros2 launch op3_manager op3_simulation.launch.py
```

#### Terminal 2: RViz Visualization
```bash
# Visual robot model with joint states
ros2 launch op3_bringup op3_bringup_visualization.launch.py
```

#### Terminal 3: GUI Demo
```bash
# GUI control interface
ros2 launch op3_gui_demo op3_demo.launch.py
```

### Option 3: Minimal Robot Display (No Control)
```bash
# Only for viewing robot model (no interaction)
ros2 launch op3_description op3_display.launch.py
```

## Command Execution Order (IMPORTANT)

### ✅ Correct Order for GUI Demo to Work:
1. **First**: Start simulation manager (`op3_manager op3_simulation.launch.py`)
2. **Second**: Start visualization (Webots or RViz) 
3. **Third**: Start GUI demo (`op3_gui_demo op3_demo.launch.py`)

### ❌ Why Previous Attempt Failed:
- Started `op3_bringup_visualization.launch.py` (visualization only)
- No robot manager running
- GUI demo had nothing to send commands to

## Testing Commands

### Verify System is Working
```bash
# Check if all required nodes are running
ros2 node list

# Should show nodes like:
# /op3_manager
# /robot_state_publisher  
# /joint_state_publisher_gui (if using RViz option)
# /op3_gui_demo
```

### Check Available Services
```bash
# Verify robotis services are available
ros2 service list | grep robotis

# Should show services like:
# /robotis/action_module/set_joint_states_msg
# /robotis/framework/set_module
# /robotis/motion_module/get_joint_states_msg
```

### Monitor Robot Activity
```bash
# Watch joint states in real-time
ros2 topic echo /joint_states

# Monitor robot status
ros2 topic echo /robotis/status
```

### Manual Command Testing
```bash
# Test init pose command manually
ros2 service call /robotis/action_module/set_joint_states_msg std_msgs/String "data: 'ini_pose'"

# Test other actions
ros2 service call /robotis/action_module/set_joint_states_msg std_msgs/String "data: 'hello'"
ros2 service call /robotis/action_module/set_joint_states_msg std_msgs/String "data: 'thank_you'"
```

## Expected Behavior When Working

### GUI Demo "Init Pose" Button:
1. **GUI Console Shows**: `[op3_gui_demo]: Go to robot initial pose.`
2. **Robot Manager Processes**: Command received and executed
3. **Visual Result**: Robot moves to initial standing position
4. **Joint States**: Published to `/joint_states` topic
5. **RViz/Webots**: Updates showing new robot pose

### GUI Demo Other Functions:
- **Walking**: Start/stop walking patterns
- **Head Control**: Pan and tilt head movements  
- **Actions**: Pre-defined gesture sequences
- **Module Control**: Enable/disable robot modules

## Package Installation Requirements

### For Webots Simulation:
```bash
# Install Webots ROS2 package
sudo apt update
sudo apt install ros-jazzy-webots-ros2
sudo apt install ros-jazzy-webots-ros2-driver
sudo apt install ros-jazzy-webots-ros2-msgs
```

### For RViz-Only Mode:
```bash
# Usually included with ROS2 desktop installation
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-rviz2
```

## Troubleshooting Commands

### If GUI Demo Not Responding:
```bash
# 1. Check if op3_manager is running
ros2 node list | grep op3_manager

# 2. If not running, start it first:
ros2 launch op3_manager op3_simulation.launch.py

# 3. Then restart GUI demo:
ros2 launch op3_gui_demo op3_demo.launch.py
```

### If No Robot Visualization:
```bash
# Check robot description parameter
ros2 param get /robot_state_publisher robot_description

# Check if joint states are being published
ros2 topic list | grep joint_states
ros2 topic hz /joint_states
```

### If X11/GUI Issues in WSL2:
```bash
# Test X11 forwarding
xclock

# Check display variable
echo $DISPLAY

# If issues, add to ~/.bashrc:
export DISPLAY=:0.0
export LIBGL_ALWAYS_INDIRECT=1
```

## Performance Notes

### Resource Usage:
- **Webots**: Higher CPU/GPU usage, full physics simulation
- **RViz-Only**: Lower resource usage, good for algorithm development
- **WSL2**: May need memory limit increase for Webots

### Recommended for Development:
1. **Start with RViz-Only** to verify setup works
2. **Move to Webots** when need physics simulation
3. **Use minimal setup** for algorithm development
4. **Full setup** for integration testing

---
*Commands verified on: September 7, 2025*
*System: WSL2 Ubuntu with ROS2 Jazzy*
*OP3 Packages: Built from jazzy-devel branches*