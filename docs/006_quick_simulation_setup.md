# Quick OP3 Simulation Setup

## Langkah Cepat untuk Simulasi ROBOTIS OP3

### 1. Setup Environment
```bash
cd ~/robotis_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 2. Jalankan Simulasi Dasar (Pilih salah satu)

#### Option A: Mode Simulasi + RViz
```bash
# Terminal 1: OP3 Manager Simulation
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 2: RViz Visualization
ros2 launch op3_description op3_display.launch.py
```

#### Option B: Test Cepat (Single Command)
```bash
# Jalankan hanya simulasi manager
ros2 launch op3_manager op3_simulation.launch.py
```

### 3. Verifikasi Simulasi Berjalan
```bash
# Check topics
ros2 topic list | grep robotis

# Monitor joint states
ros2 topic echo /robotis/present_joint_states --once

# Check services
ros2 service list | grep robotis
```

### 4. Test Basic Control
```bash
# Set ke base module
ros2 service call /robotis/set_module robotis_controller_msgs/srv/SetModule "{module_name: 'base_module'}"

# Test joint movement
ros2 topic pub /robotis/sync_write_item robotis_controller_msgs/msg/SyncWriteItem "{item_name: 'goal_position', joint_name: ['head_pan'], value: [0.3]}" --once
```

## Quick Troubleshooting

### Jika ada error "Error Set port":
✅ **Normal** - Ini expected dalam mode simulasi karena tidak ada hardware

### Jika ada error camera:
```bash
# Buat dummy camera calibration
mkdir -p ~/.ros/camera_info
echo "image_width: 640
image_height: 480
camera_name: camera" > ~/.ros/camera_info/camera.yaml
```

### Jika RViz tidak menampilkan robot:
```bash
# Check robot description loaded
ros2 topic echo /robot_description --once

# Re-launch display
ros2 launch op3_description op3_display.launch.py
```

## Expected Output (Success)

Ketika berhasil, Anda akan melihat:
- ✅ `[INFO] [op3_manager]: manager->init`
- ✅ `simulation: True` dalam parameter
- ✅ Joint states topics aktif
- ✅ Model OP3 tampil di RViz (jika dijalankan)

## Stop Simulation
```bash
# Ctrl+C pada terminal yang menjalankan launch files
# Atau gunakan:
pkill -f op3_manager
pkill -f rviz2
```

---

**📝 Note:** Guide lengkap tersedia di `docs/op3_simulation_guide.md`