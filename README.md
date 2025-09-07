# ROBOTIS OP3 ROS2 Workspace

A complete ROS 2 Jazzy workspace for ROBOTIS OP3 humanoid robot development and simulation.

## 🤖 Quick Start

### Prerequisites
- Ubuntu 22.04/24.04 (or WSL2)
- ROS 2 Jazzy installed
- 8GB+ RAM recommended

### Build Workspace
```bash
# Clone this workspace
git clone <your-repo-url> ~/robotis_ws
cd ~/robotis_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

### Quick Test (GUI Demo without Hardware)
```bash
# Terminal 1: Start simulation manager
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 2: Start visualization
ros2 launch op3_bringup op3_bringup_visualization.launch.py

# Terminal 3: Start GUI demo
ros2 launch op3_gui_demo op3_demo.launch.py
```

Click "Init Pose" in the GUI demo to see the robot move!

## 📋 What's Included

**44 ROS 2 packages** organized in 8 repository groups providing complete OP3 functionality:

### 🏗️ Core Framework (4 packages)
- **ROBOTIS-Framework**: Base controller, device interfaces, common utilities
- **ROBOTIS-Framework-msgs**: Framework message definitions
- **ROBOTIS-Math**: Kinematics and mathematical functions
- **DynamixelSDK**: Multi-language servo communication (C++, Python, Java, MATLAB)

### 🤖 Robot Control Core (11 packages)
- **op3_manager** - Central robot manager and configuration hub ⭐
- **op3_base_module** - Base functionality and initialization  
- **op3_action_module** - Pre-defined motion sequences and playback
- **op3_walking_module** - Walking pattern generation and control
- **op3_online_walking_module** - Real-time walking adjustments
- **op3_balance_control** - Dynamic balance control using IMU
- **op3_head_control_module** - Head pan/tilt movement control
- **op3_direct_control_module** - Direct joint control interface
- **op3_tuning_module** - Parameter tuning and calibration
- **op3_kinematics_dynamics** - Forward/inverse kinematics
- **open_cr_module** - OpenCR sub-controller communication

### 📝 Messages & Description (8 packages) 
- **op3_description** - Robot URDF, meshes, visual models, RViz configs
- **op3_*_msgs** - 7 message packages for different modules (action, walking, ball detection, etc.)

### 🎮 Demo & Applications (4 packages)
- **op3_bringup** - Launch files and robot startup configurations ⭐
- **op3_gui_demo** - Main GUI interface for robot control ⭐
- **op3_ball_detector** - Soccer ball detection and tracking
- **op3_demo** - Basic demonstration modes

### 🛠️ Development Tools (8 packages)
- **op3_action_editor** - Motion sequence creation and editing
- **op3_offset_tuner_client/server** - Joint offset calibration tools
- **op3_tuner_client** - General parameter tuning interface
- **op3_camera_setting_tool** - Camera parameter configuration
- **op3_web_setting_tool** - Web-based configuration interface
- **op3_navigation** - Navigation and path planning

### 👁️ Vision & Utilities (4 packages)
- **face_detection** - OpenCV-based face recognition and tracking
- **usb_cam** - Camera interface and configuration
- **ros_madplay_player** - Audio playback using madplay
- **ros_mpg321_player** - MP3 audio playback

### ⚙️ Hardware Specifications
- **20 DOF**: Arms (6), Legs (12), Head (2) + IMU sensor
- **Servos**: XM430-W350 actuators with Protocol 2.0  
- **Controller**: OpenCR sub-controller for sensors
- **Communication**: `/dev/ttyUSB0` at 2,000,000 bps

## 🎯 Use Cases

### 1. Hardware Robot Operation
```bash
# For actual OP3 robot
ros2 launch op3_bringup op3_bringup.launch.py
ros2 launch op3_gui_demo op3_demo.launch.py
```

### 2. Simulation Development (WSL2-friendly)
```bash
# Webots simulation (recommended)
ros2 launch op3_webots_ros2 robot_launch.py
ros2 launch op3_manager op3_simulation.launch.py
ros2 launch op3_gui_demo op3_demo.launch.py

# RViz-only visualization
ros2 launch op3_manager op3_simulation.launch.py
ros2 launch op3_bringup op3_bringup_visualization.launch.py
ros2 launch op3_gui_demo op3_demo.launch.py
```

### 3. Motion Development
```bash
# Edit motion sequences
ros2 run op3_action_editor op3_action_editor

# Tune parameters
ros2 run op3_tuner_client op3_tuner_client
```

## 📚 Documentation

All documentation is organized chronologically in `docs/`:

| File | Description |
|------|-------------|
| [001_increase_swap.md](docs/001_increase_swap.md) | WSL2 swap space setup |
| [002_cyclonedds_fix.md](docs/002_cyclonedds_fix.md) | ROS2 middleware fixes |
| [003_hardware_connection_guide.md](docs/003_hardware_connection_guide.md) | Hardware setup guide |
| [004_op3_troubleshooting.md](docs/004_op3_troubleshooting.md) | General troubleshooting |
| [005_op3_simulation_guide.md](docs/005_op3_simulation_guide.md) | Basic simulation setup |
| [006_quick_simulation_setup.md](docs/006_quick_simulation_setup.md) | Fast setup reference |
| [007_wslg_setup_guide.md](docs/007_wslg_setup_guide.md) | Windows 11 WSLg setup |
| [008_op3_simulation_research_findings.md](docs/008_op3_simulation_research_findings.md) | ROS2 Jazzy compatibility research |
| [009_webots_simulation_setup_guide.md](docs/009_webots_simulation_setup_guide.md) | Complete Webots setup |
| [010_working_simulation_commands.md](docs/010_working_simulation_commands.md) | ✅ **TESTED COMMANDS** |
| [011_wsl2_x11_forwarding_setup.md](docs/011_wsl2_x11_forwarding_setup.md) | X11 GUI setup for WSL2 |

> 💡 **Start with [010_working_simulation_commands.md](docs/010_working_simulation_commands.md)** for tested, working commands!

## 🚀 Simulation Options

### Option 1: Webots (Full Physics) ⭐ **Recommended**
- Complete physics simulation
- Official ROBOTIS support
- Works great in WSL2 with X11

### Option 2: RViz-Only (Lightweight)
- Fast startup, low resource usage
- Perfect for algorithm development
- No physics engine required

### Option 3: Hardware (Real Robot)
- Full robot functionality
- Requires actual OP3 hardware

## 🛠️ Development Environment

### WSL2 Users
This workspace is **WSL2-friendly**! See our guides:
- [X11 Forwarding Setup](docs/011_wsl2_x11_forwarding_setup.md)
- [Webots on WSL2](docs/009_webots_simulation_setup_guide.md)

### Memory Requirements
- **Minimum**: 8GB RAM
- **Recommended**: 16GB RAM
- **Large builds**: Use [swap space setup](docs/001_increase_swap.md)

## 🎮 Available Demo Modes

### GUI Demo Functions
- **Init Pose** - Move robot to initial standing position
- **Walking** - Start/stop walking patterns
- **Head Control** - Pan and tilt head movements
- **Actions** - Pre-defined gesture sequences (hello, thank you, etc.)

### Specialized Demos
- **Soccer Demo** - Ball detection and kicking
- **Vision Demo** - Face tracking and following
- **Action Demo** - Motion sequence demonstration

## 🔧 Build Tips

### Parallel Build Optimization
```bash
# Standard build
colcon build

# Fast build (if you have enough RAM)
colcon build --parallel-workers 4

# Memory-limited build
colcon build --parallel-workers 2

# Single package
colcon build --packages-select op3_gui_demo
```

### Clean Build (if issues)
```bash
rm -rf build/ install/ log/
colcon build
```

## ❗ Troubleshooting

### GUI Demo Not Responding?
**Problem**: GUI demo buttons don't work
**Solution**: Make sure `op3_manager` is running first!

```bash
# ✅ Correct order:
# 1. Start manager first
ros2 launch op3_manager op3_simulation.launch.py

# 2. Then start GUI demo
ros2 launch op3_gui_demo op3_demo.launch.py
```

### No Robot Visualization?
**Problem**: Robot model not showing in RViz
**Solution**: Start visualization launch file

```bash
ros2 launch op3_bringup op3_bringup_visualization.launch.py
```

### WSL2 GUI Issues?
**Problem**: GUI applications won't start
**Solution**: Check X11 forwarding

```bash
# Test GUI
xclock

# Fix display if needed
export DISPLAY=:0.0
```

## 📞 Support

### Quick Reference
- **Hardware specs**: Intel NUC i3, 20 DOF, 3.5kg, 510mm height
- **ROS Version**: ROS 2 Jazzy
- **Build System**: colcon with ament_cmake
- **Key packages**: 30+ packages for complete OP3 functionality

### Documentation Priority
1. 📋 **[Working Commands](docs/010_working_simulation_commands.md)** - Start here!
2. 🔍 **[Research Findings](docs/008_op3_simulation_research_findings.md)** - Understanding simulation options
3. 🖥️ **[WSL2 Setup](docs/011_wsl2_x11_forwarding_setup.md)** - GUI setup for Windows users
4. 🤖 **[Webots Guide](docs/009_webots_simulation_setup_guide.md)** - Full simulation setup

---

**🎯 Ready to start?** Try the Quick Test commands above, then explore the numbered documentation guides!

*Built with ❤️ for ROBOTIS OP3 development*