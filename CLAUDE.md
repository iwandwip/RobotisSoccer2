# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview
This is a **ROBOTIS OP3 workspace** - a ROS 2 workspace containing packages for the ROBOTIS OP3 humanoid robot platform. The workspace includes:
- Robot control modules and framework
- Hardware interfaces (Dynamixel SDK)
- Vision processing (face detection, camera)
- Motion modules (walking, action, balance)
- Development tools and utilities

## Hardware Specifications

### Physical Dimensions
- **Height**: 510mm
- **Weight**: 3.5kg (without skin cover)
- **Degrees of Freedom (DOF)**: 20
- **Actuators**: DYNAMIXEL XM430-W350-R servos

### Main Hardware Components
- **Main Controller**: Intel NUC i3 processor
  - Dual-core processor
  - 8GB DDR4 RAM
  - 250GB M.2 SSD
  - HDMI and DisplayPort support
- **Sub-Controller**: OpenCR board
- **Camera**: Logitech C920 HD Pro Webcam
- **IMU Sensor**: 3-axis Gyroscope, Accelerometer, Magnetometer
- **Battery**: Lipo 3-cell 11.1v 3300mA

### Key Improvements from OP2
- Upgraded actuators with higher torque
- Increased computational power
- Improved RAM and SSD capacity
- ROS 2 Jazzy compatibility

## Build System
This project uses **ROS 2 colcon** build system with **ament_cmake**.

### ROBOTIS CLI Tool (Recommended)
The workspace includes a comprehensive Python CLI tool for streamlined operations:

```bash
# Activate virtual environment (required)
source venv/bin/activate

# Launch the CLI tool
python3 robotis_cli.py
# or
./robotis_cli.py
```

**CLI Features:**
- **🔨 Colcon Build Options**: Interactive build configurations with parallel worker control
- **🧹 Clean Workspace**: Selective cleanup of build/install/log directories with size analysis
- **🗂️ Git Repository Cleanup**: Management of nested .git repositories for monorepo conversion
- **Real-time output**: Live build progress with timing and error handling
- **Safety confirmations**: Prevents accidental operations with user prompts

### Traditional Commands (Manual)
```bash
# Build entire workspace
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with parallel jobs (adjust based on memory)
colcon build --parallel-workers 4

# Source the workspace
source install/setup.bash

# Clean build
rm -rf build/ install/ log/
colcon build
```

### CLI Tool Setup
```bash
# Install required packages (one-time setup)
sudo apt install python3.12-venv python3-full -y

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install inquirer

# Run the tool
python3 robotis_cli.py
```

### Memory Management
- **Important**: Large builds may require increased swap space
- WSL2 users: see `increase_swap.md` for 8GB swap setup instructions
- Use `--parallel-workers 2` on systems with limited RAM

## Workspace Architecture

This workspace contains **44 ROS 2 packages** organized in 8 main repository groups:

### Core Framework Structure
- **ROBOTIS-Framework** (3 packages): Base controller and device abstraction layer
  - `robotis_controller`: Main robot controller framework with ROS2 interface
  - `robotis_device`: Hardware device interfaces and abstractions
  - `robotis_framework_common`: Shared utilities and common functions

- **ROBOTIS-Framework-msgs** (1 package): Framework message definitions
  - `robotis_controller_msgs`: Core framework message types

### Hardware Integration Layer
- **DynamixelSDK** (Multi-language SDK): Low-level motor communication
  - C/C++, Python, Java, MATLAB support
  - ROS 2 interface packages included
  - Protocol 2.0 support for XM430-W350 servos

- **ROBOTIS-Math** (1 package): Mathematical utilities
  - `robotis_math`: Kinematics and mathematical functions

### Robot-Specific Core (ROBOTIS-OP3)
**Main control packages (11 packages):**
- `op3_manager`: Central robot manager and configuration hub
- `op3_base_module`: Base functionality and initialization
- `op3_action_module`: Pre-defined action sequences and playback
- `op3_walking_module`: Walking pattern generation and control
- `op3_online_walking_module`: Real-time walking adjustments
- `op3_balance_control`: Dynamic balance control using IMU
- `op3_head_control_module`: Head pan/tilt movement control
- `op3_direct_control_module`: Direct joint control interface
- `op3_tuning_module`: Parameter tuning and calibration
- `op3_kinematics_dynamics`: Forward/inverse kinematics
- `op3_localization`: Robot localization algorithms
- `open_cr_module`: OpenCR sub-controller communication

### Robot Description & Messages
- **ROBOTIS-OP3-Common** (1 package):
  - `op3_description`: Robot URDF, meshes, visual models, RViz configs

- **ROBOTIS-OP3-msgs** (7 packages): Custom message definitions
  - `op3_action_module_msgs`: Action sequence messages
  - `op3_walking_module_msgs`: Walking control messages  
  - `op3_online_walking_module_msgs`: Online walking messages
  - `op3_ball_detector_msgs`: Soccer ball detection messages
  - `op3_camera_setting_tool_msgs`: Camera configuration messages
  - `op3_offset_tuner_msgs`: Joint offset tuning messages
  - `op3_tuning_module_msgs`: Parameter tuning messages

### Demo Applications (ROBOTIS-OP3-Demo)
**Demo and testing packages (4 packages):**
- `op3_bringup`: Launch files and robot startup configurations
- `op3_demo`: Basic demonstration modes
- `op3_ball_detector`: Soccer ball detection and tracking
- `op3_read_write_demo`: Read/write operation examples

### Development Tools (ROBOTIS-OP3-Tools)
**GUI and configuration tools (8 packages):**
- `op3_gui_demo`: Main GUI interface for robot control
- `op3_action_editor`: Motion sequence creation and editing
- `op3_offset_tuner_client/server`: Joint offset calibration tools
- `op3_tuner_client`: General parameter tuning interface
- `op3_camera_setting_tool`: Camera parameter configuration
- `op3_web_setting_tool`: Web-based configuration interface
- `op3_navigation`: Navigation and path planning

### Vision & Utilities (ROBOTIS-OP3-ETC)
**Additional functionality (2 packages):**
- `face_detection`: OpenCV-based face recognition and tracking
- `usb_cam`: Camera interface and configuration

- **ROBOTIS-Utility** (2 packages): Audio and media utilities
  - `ros_madplay_player`: Audio playback using madplay
  - `ros_mpg321_player`: MP3 audio playback

### Hardware Configuration
**20 Degrees of Freedom mapping:**
```
Joint Configuration (ID: Joint Name):
Arms:  1-6   (r/l_sho_pitch, r/l_sho_roll, r/l_el)
Legs:  7-18  (r/l_hip_yaw, r/l_hip_roll, r/l_hip_pitch, r/l_knee, r/l_ank_pitch, r/l_ank_roll)  
Head:  19-20 (head_pan, head_tilt)
Sensor: 200  (open-cr IMU/button controller)
```

**Communication Protocol:**
- **Device**: `/dev/ttyUSB0` (configurable)
- **Baudrate**: 2,000,000 bps
- **Protocol**: Dynamixel Protocol 2.0
- **Actuators**: XM430-W350 servos (all joints)
- **Sub-controller**: OpenCR board for IMU/sensors

## Robot Bringup and Operation

### Official Bringup Procedure
```bash
# Stop default demo service (if running)
sudo systemctl stop op3_demo.service

# Launch main robot bringup
ros2 launch op3_bringup op3_bringup.launch.py

# Alternative: Launch with manager
ros2 launch op3_manager op3_manager.launch.py
```

### Demo Modes
The OP3 supports multiple demo modes for different applications:

- **Soccer Mode**: Searches for and kicks a ball using color detection
- **Vision Mode**: Tracks and follows detected faces using camera
- **Action Mode**: Performs predefined motion sequences with audio feedback

### Launch Files
Launch files use **Python-based ROS 2 launch system**:
- **Main robot bringup**: `ros2 launch op3_bringup op3_bringup.launch.py`
- **Robot manager**: `ros2 launch op3_manager op3_manager.launch.py`
- **Face tracking**: `ros2 launch face_detection face_tracking.launch.py`
- **Camera**: `ros2 launch usb_cam camera.launch.py`

### Visualization
- **RViz2**: `rviz2` - Robot state visualization
- **Joint State Publisher**: Publishes joint positions
- **Robot State Publisher**: Publishes robot transforms

## Development Patterns

### Package Structure
```
package_name/
├── package.xml          # ROS 2 package metadata
├── CMakeLists.txt      # Build configuration
├── src/                # C++ source files
├── include/            # Header files
├── launch/             # Launch files (.launch.py)
├── config/             # Configuration files (.yaml)
└── msg/srv/            # Message/service definitions
```

### Key Configuration Files
- **Robot configuration**: `op3_manager/config/OP3.robot`
- **Joint offsets**: `op3_manager/config/offset.yaml`
- **Dynamixel initialization**: `op3_manager/config/dxl_init_OP3.yaml`

### Code Conventions
- **C++ Standard**: Modern C++ with ROS 2 APIs
- **Headers**: Include ROBOTIS copyright header
- **Naming**: Snake_case for ROS conventions
- **Architecture**: Module-based design with clear separation

### Hardware Constants
```cpp
const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
```

## Development Tools

### ROBOTIS OP3 Specific Tools
- **Action Editor**: GUI tool for creating and editing custom motion sequences
- **Tuner Client**: Tool for adjusting joint offsets and PID gains in real-time
- **Web Setting Tool**: Browser-based parameter configuration interface
- **Ball Detector**: Color-based object detection and tracking utility

### Motion Development
- **Action sequences**: Pre-defined motions stored in action files
- **Walking patterns**: Dynamic walking gait generation
- **Balance control**: Real-time balance adjustment using IMU feedback
- **Joint offset tuning**: Fine-tuning for mechanical variations

### Vision Processing
- **Face detection**: OpenCV-based face recognition and tracking
- **Ball detection**: Color-based ball detection for soccer applications
- **Camera calibration**: Tools for camera parameter adjustment

## Testing and Validation
- **Build verification**: Always run `colcon build` after changes
- **Hardware testing**: Test with actual robot hardware when available
- **Simulation**: Some packages support Gazebo simulation mode
- **Message validation**: Test custom message generation with `colcon build --packages-select <msg_package>`
- **Motion testing**: Use Action Editor to test motion sequences safely
- **Vision testing**: Verify camera functionality and detection algorithms

## Common Issues
- **Memory limitations**: Use swap space for large builds
- **Device permissions**: Ensure USB device access (`/dev/ttyUSB0`)
- **ROS 2 environment**: Always source workspace after building
- **Parallel builds**: Reduce parallel workers if encountering memory issues

## Documentation
Comprehensive guides and troubleshooting documentation available in `docs/` (numbered chronologically):

### System Setup (001-003)
- **[001 Increase Swap](docs/001_increase_swap.md)**: WSL2 swap space configuration for large builds
- **[002 CycloneDDS Fix](docs/002_cyclonedds_fix.md)**: Resolve RMW middleware configuration errors
- **[003 Hardware Connection Guide](docs/003_hardware_connection_guide.md)**: Serial port, camera, and device connectivity

### Core Troubleshooting (004-007)
- **[004 OP3 Troubleshooting](docs/004_op3_troubleshooting.md)**: Complete troubleshooting reference
- **[005 OP3 Simulation Guide](docs/005_op3_simulation_guide.md)**: General simulation setup guide
- **[006 Quick Simulation Setup](docs/006_quick_simulation_setup.md)**: Fast simulation setup reference
- **[007 WSLg Setup Guide](docs/007_wslg_setup_guide.md)**: Windows 11 WSLg configuration

### Advanced Simulation & Development (008-011)
- **[008 OP3 Simulation Research Findings](docs/008_op3_simulation_research_findings.md)**: Research findings on ROS2 Jazzy simulation compatibility
- **[009 Webots Simulation Setup Guide](docs/009_webots_simulation_setup_guide.md)**: Complete Webots simulation setup for WSL2
- **[010 Working Simulation Commands](docs/010_working_simulation_commands.md)**: Verified commands for GUI demo testing without hardware
- **[011 WSL2 X11 Forwarding Setup](docs/011_wsl2_x11_forwarding_setup.md)**: X11 GUI forwarding configuration for WSL2

### Key Fixes Applied
- **CycloneDDS Configuration**: Updated `CYCLONEDDS_URI` in `~/.bashrc` from `/home/robotis/.ros/cyclonedds.xml` to `/home/iwandwp/.ros/cyclonedds.xml`
- **Environment Setup**: ROS 2 Jazzy configured with proper middleware settings
- **Workspace Structure**: All documentation organized in `docs/` folder with chronological numbering

## System Requirements and Installation

### Recommended System
- **Operating System**: Linux Mint 22 Xfce (recommended) or Ubuntu 22.04/24.04
- **ROS Version**: ROS 2 Jazzy
- **Memory**: Minimum 8GB RAM (16GB recommended for development)
- **Storage**: Minimum 50GB free space
- **Network**: WiFi capability for robot communication

### Installation Components
- **DYNAMIXEL SDK**: Low-level servo communication
- **ROBOTIS Framework**: Core robot control framework
- **OP3-specific ROS packages**: Robot-specific functionality
- **OpenCV**: Vision processing library
- **Additional tools**: Action Editor, Web Setting Tool, Tuner Client

### Network Configuration
- Configure WiFi hotspot for robot communication
- Set up SSH access for remote development
- Enable desktop sharing for remote GUI access
- Configure Samba for file sharing (optional)

### Recovery and Maintenance
- **Recovery method**: CloneZilla USB recovery images
- **Backup recommendation**: Create system images before major changes
- **Updates**: Regular ROS package updates using `apt` or `rosdep`

## Safety and Maintenance Guidelines

### Safety Precautions
- **Age restriction**: Not suitable for children under 15 years
- **Handling**: Use careful handling procedures to avoid damage
- **Pinch points**: Be aware of potential pinch points between moving parts
- **Power management**: Always ensure proper battery handling and charging
- **Workspace safety**: Maintain clear workspace during robot operation

### Maintenance Requirements
- **Regular inspection**: Check joints and connections periodically
- **Gear maintenance**: Regular gear inspection and lubrication as needed
- **Battery care**: Monitor battery health and charging cycles
- **Calibration**: Periodic joint offset and sensor calibration
- **Cleaning**: Keep camera lens and sensors clean for optimal performance

### Operational Guidelines
- **Power on sequence**: Follow proper startup procedures
- **Emergency stop**: Know how to quickly stop robot motion if needed
- **Environment**: Operate in appropriate lighting and surface conditions
- **Monitoring**: Always supervise robot during autonomous operation