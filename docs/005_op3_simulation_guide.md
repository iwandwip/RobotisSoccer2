# ROBOTIS OP3 Simulation Guide (Tanpa Hardware)

## Overview
Panduan ini menjelaskan cara menjalankan simulasi ROBOTIS OP3 tanpa memerlukan hardware fisik. Anda dapat menggunakan RViz untuk visualisasi dan Gazebo untuk simulasi lengkap.

## Prerequisites
- ROS 2 Jazzy sudah terinstall
- ROBOTIS OP3 workspace sudah di-build (`colcon build`)
- Environment sudah di-source (`source install/setup.bash`)

## Method 1: Simulasi Dasar dengan op3_manager

### Langkah 1: Test Mode Simulasi
```bash
# Jalankan op3_manager dalam mode simulasi
ros2 launch op3_manager op3_simulation.launch.py
```

**Yang terjadi:**
- Parameter `simulation: True` mengaktifkan mode simulasi
- Tidak ada komunikasi ke hardware fisik
- Joint states dipublikasi secara virtual

### Langkah 2: Visualisasi dengan RViz
```bash
# Di terminal baru, jalankan RViz display
ros2 launch op3_description op3_display.launch.py
```

**Yang akan Anda lihat:**
- Model 3D ROBOTIS OP3 di RViz
- Joint states real-time
- Transform frames robot

### Langkah 3: Monitor Topics
```bash
# Lihat topic yang tersedia
ros2 topic list

# Monitor joint states
ros2 topic echo /robotis/present_joint_states

# Monitor robot status
ros2 topic echo /robotis/status
```

## Method 2: Simulasi dengan RViz Lengkap

### Setup Simulasi Bringup
Untuk menghindari error camera dan hardware:

```bash
# Jalankan hanya komponen simulasi
ros2 launch op3_manager op3_simulation.launch.py &
ros2 launch op3_description op3_display.launch.py
```

### Control Robot dalam Simulasi
```bash
# Aktifkan modules untuk simulasi
ros2 service call /robotis/set_module robotis_controller_msgs/srv/SetModule "{module_name: 'none'}"

# Set ke posisi awal
ros2 service call /robotis/set_module robotis_controller_msgs/srv/SetModule "{module_name: 'base_module'}"

# Aktifkan action module
ros2 service call /robotis/set_module robotis_controller_msgs/srv/SetModule "{module_name: 'action_module'}"
```

## Method 3: Simulasi Gazebo (Advanced)

### Install Dependencies
```bash
# Install ROS 2 Control packages
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control

# Install Gazebo simulation (jika tersedia)
sudo apt install ros-jazzy-gazebo-*
```

### Launch Gazebo Simulation
```bash
# Method 1: Jika ada paket op3_gazebo
ros2 launch op3_gazebo robot_sim.launch.py

# Method 2: Spawn manual di Gazebo
gazebo &
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity op3
ros2 launch op3_manager op3_simulation.launch.py
```

## Troubleshooting

### Error: "Error Set port" / "Unknown error code"
**Solusi:** Error ini normal dalam mode simulasi karena tidak ada hardware.
```bash
# Pastikan menggunakan launch file simulasi
ros2 launch op3_manager op3_simulation.launch.py
```

### Error: Camera calibration file not found
**Solusi:** Buat dummy camera calibration atau disable camera:
```bash
# Buat directory
mkdir -p ~/.ros/camera_info

# Buat dummy calibration file
echo "image_width: 640
image_height: 480
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [500, 0, 320, 0, 500, 240, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0, 0, 0, 0, 0]" > ~/.ros/camera_info/camera.yaml
```

### Error: "/sys/class/video4linux/" not found
**Solusi:** Error ini terjadi karena tidak ada camera. Gunakan simulasi tanpa camera atau install virtual camera.

## Testing Simulasi

### Test 1: Basic Joint Movement
```bash
# Terminal 1: Jalankan simulasi
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 2: RViz
ros2 launch op3_description op3_display.launch.py

# Terminal 3: Test joint movement
ros2 topic pub /robotis/sync_write_item robotis_controller_msgs/msg/SyncWriteItem "{
  item_name: 'goal_position',
  joint_name: ['r_sho_pitch'],
  value: [0.5]
}" --once
```

### Test 2: Walking Module (Simulation)
```bash
# Aktifkan walking module
ros2 service call /robotis/set_module robotis_controller_msgs/srv/SetModule "{module_name: 'walking_module'}"

# Start walking
ros2 topic pub /robotis/walking/command std_msgs/msg/String "data: 'start'" --once
```

### Test 3: Action Module
```bash
# Load action file
ros2 service call /robotis/action/load_action robotis_controller_msgs/srv/LoadAction "{file_path: '/path/to/action/file'}"

# Play action
ros2 topic pub /robotis/action/page_num std_msgs/msg/Int32 "data: 1" --once
```

## Useful Commands

### Monitor System Status
```bash
# Check running nodes
ros2 node list

# Check node info
ros2 node info /op3_manager

# Check available services
ros2 service list

# Check joint states
ros2 topic echo /joint_states
```

### Debug Topics
```bash
# List all topics
ros2 topic list

# Check topic info
ros2 topic info /robotis/present_joint_states

# Record topics for analysis
ros2 bag record -a
```

## Configuration Files

### Important Files untuk Simulasi
- `op3_manager/config/OP3.robot` - Robot configuration
- `op3_manager/config/offset.yaml` - Joint offsets
- `op3_manager/config/dxl_init_OP3.yaml` - Motor initialization
- `op3_description/urdf/robotis_op3.urdf.xacro` - Robot URDF model

### Launch Files untuk Simulasi
- `op3_manager/launch/op3_simulation.launch.py` - Manager dalam mode simulasi
- `op3_description/launch/op3_display.launch.py` - RViz display
- `op3_bringup/launch/op3_bringup.launch.py` - Full bringup (hardware mode)

## Tips & Best Practices

1. **Always use simulation launch files** untuk menghindari hardware errors
2. **Start with RViz visualization** untuk memastikan model loaded correctly
3. **Monitor joint states** untuk memverifikasi simulasi berjalan
4. **Use separate terminals** untuk setiap komponen simulasi
5. **Check logs** jika ada error: `~/.ros/log/latest/`

## Next Steps

- Experiment dengan joint control dalam simulasi
- Implementasi custom walking patterns
- Integrasi dengan computer vision (virtual camera)
- Development behavior modules untuk simulasi

## References

- [ROBOTIS OP3 e-Manual](https://emanual.robotis.com/docs/en/platform/op3/simulation/)
- [ROS 2 Gazebo Integration](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [ROBOTIS Framework Documentation](https://github.com/ROBOTIS-GIT/ROBOTIS-Framework)