# ROBOTIS OP3 Troubleshooting Guide

## Common Issues and Solutions

### 1. Build Issues

#### Memory Errors During Build
```bash
# Error: virtual memory exhausted: Cannot allocate memory
```
**Solution:**
```bash
# Increase swap space (see increase_swap.md)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Build with limited parallel jobs
colcon build --parallel-workers 2
```

#### Missing Dependencies
```bash
# Error: Could not find a package configuration file
```
**Solution:**
```bash
# Install missing ROS 2 packages
sudo apt update
sudo apt install ros-jazzy-<package-name>

# Install workspace dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Runtime Issues

#### CycloneDDS Configuration Error
See [cyclonedds_fix.md](cyclonedds_fix.md) for detailed solution.

#### Hardware Connection Errors
See [hardware_connection_guide.md](hardware_connection_guide.md) for hardware-specific issues.

#### Permission Errors
```bash
# Error: Permission denied accessing /dev/ttyUSB0
```
**Solution:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login, or run:
newgrp dialout
```

### 3. Launch File Issues

#### Node Not Found
```bash
# Error: Package 'package_name' not found
```
**Solution:**
```bash
# Source workspace
source ~/robotis_ws/install/setup.bash

# Rebuild specific package
colcon build --packages-select package_name
```

#### Parameter File Not Found
```bash
# Error: Parameter file does not exist
```
**Solution:**
```bash
# Check parameter file paths
find ~/robotis_ws -name "*.yaml" | grep config

# Verify launch file parameter paths
ros2 pkg prefix op3_manager
```

### 4. Network Issues

#### ROS Domain Issues
```bash
# Nodes not discovering each other
```
**Solution:**
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Set consistent domain ID
export ROS_DOMAIN_ID=1

# Check network configuration
ros2 doctor --report
```

#### Multicast Issues
```bash
# Nodes timeout on discovery
```
**Solution:**
Configure CycloneDDS for localhost-only:
```xml
<!-- ~/.ros/cyclonedds.xml -->
<General>
  <Interfaces>
    <NetworkInterface name="lo" />
  </Interfaces>
  <AllowMulticast>false</AllowMulticast>
</General>
```

### 5. Development Tools

#### Useful Commands
```bash
# Check package installation
ros2 pkg list | grep op3

# List available launch files
ros2 pkg prefix op3_bringup
find $(ros2 pkg prefix op3_bringup) -name "*.launch.py"

# Monitor topics
ros2 topic list
ros2 topic echo /robotis/present_joint_states

# Check node status
ros2 node list
ros2 node info /op3_manager

# Debug services
ros2 service list
ros2 service call /robotis/get_present_joint_ctrl_modules robotis_controller_msgs/srv/GetJointModule
```

#### Log Analysis
```bash
# Find log files
ls ~/.ros/log/

# Monitor logs in real-time
tail -f ~/.ros/log/latest/op3_manager/stdout.log

# Search for errors
grep -r "ERROR" ~/.ros/log/latest/
```

### 6. Environment Setup

#### Complete Environment Check
```bash
# Check all ROS 2 environment variables
env | grep ROS

# Verify workspace sourcing
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep robotis_ws

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))" | grep robotis
```

#### Clean Environment Setup
```bash
# Start fresh terminal
# Source ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source workspace
source ~/robotis_ws/install/setup.bash

# Set required environment variables
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/iwandwp/.ros/cyclonedds.xml
```

### 7. Performance Issues

#### High CPU Usage
```bash
# Monitor system resources
htop
ros2 run rqt_top rqt_top
```

#### Memory Leaks
```bash
# Monitor memory usage
ros2 run rqt_runtime_monitor rqt_runtime_monitor

# Check for memory leaks
valgrind --tool=memcheck ros2 run op3_manager op3_manager
```

### 8. Recovery Procedures

#### Complete Rebuild
```bash
# Clean build
rm -rf ~/robotis_ws/build/ ~/robotis_ws/install/ ~/robotis_ws/log/

# Rebuild everything
cd ~/robotis_ws
colcon build --parallel-workers 2
```

#### Reset Configuration
```bash
# Backup current config
cp ~/.bashrc ~/.bashrc.backup

# Reset ROS 2 environment
# Edit ~/.bashrc and remove ROS 2 related exports
# Re-source ROS 2 installation
```

#### Package-Specific Rebuild
```bash
# Rebuild specific package and dependencies
colcon build --packages-up-to op3_manager

# Build only specific package
colcon build --packages-select op3_manager
```

## Quick Diagnostic Checklist

- [ ] Workspace built successfully: `colcon build`
- [ ] Environment sourced: `source install/setup.bash`
- [ ] CycloneDDS configured: `echo $CYCLONEDDS_URI`
- [ ] Hardware connected: `ls /dev/ttyUSB*`
- [ ] Permissions set: `groups | grep dialout`
- [ ] ROS 2 working: `ros2 doctor`
- [ ] Launch file exists: `ros2 launch op3_bringup op3_bringup.launch.py --show-args`

## Getting Help

### Official Resources
- [ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/platform/op3/)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [ROBOTIS GitHub](https://github.com/ROBOTIS-GIT)

### Community Support
- [ROS Answers](https://answers.ros.org/)
- [ROBOTIS OpenSource](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- [ROS 2 Discourse](https://discourse.ros.org/)