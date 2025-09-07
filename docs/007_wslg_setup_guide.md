# WSLg Setup Guide untuk ROBOTIS OP3 Development

## Apa itu WSLg?
WSLg (Windows Subsystem for Linux GUI) memungkinkan aplikasi Linux GUI berjalan native di Windows. Sangat penting untuk RViz, Gazebo, dan tools robotics lainnya.

## Quick Check - Apakah WSLg Sudah Aktif?

```bash
# Check WSLg status
echo $DISPLAY
# Should output: :0 (jika aktif)

# Test basic GUI
xcalc
# Jika muncul calculator window = WSLg aktif ✅
```

## Aktivasi WSLg (Jika Belum Aktif)

### 1. Update WSL (di Windows PowerShell)
```powershell
# Update WSL ke versi terbaru
wsl --update

# Restart WSL
wsl --shutdown
wsl
```

### 2. Verifikasi Environment Variables
```bash
# Check display variables
echo $DISPLAY        # Should be :0
echo $WAYLAND_DISPLAY # Should be wayland-0

# If empty, set manually:
export DISPLAY=:0
echo 'export DISPLAY=:0' >> ~/.bashrc
```

### 3. Install GUI Dependencies
```bash
# Install basic GUI tools
sudo apt update
sudo apt install x11-apps

# Install ROS 2 GUI tools
sudo apt install ros-jazzy-rqt* ros-jazzy-rviz2
```

## Testing WSLg dengan OP3

### Test 1: Basic GUI
```bash
# Test calculator (should open window)
xcalc

# Test Firefox (advanced test)
sudo apt install firefox
firefox
```

### Test 2: RViz Visualization
```bash
cd ~/robotis_ws
source install/setup.bash

# Launch RViz (should open GUI window)
rviz2

# Test OP3 model visualization
ros2 launch op3_description op3_display.launch.py
```

### Test 3: RQT Tools
```bash
# GUI debugging tools
rqt
rqt_graph
rqt_image_view
```

## Troubleshooting

### Problem: "cannot connect to X server"
```bash
# Solution 1: Reset WSL
wsl --shutdown (di PowerShell Windows)
wsl

# Solution 2: Set DISPLAY manually
export DISPLAY=:0.0
source ~/.bashrc
```

### Problem: Black screen atau GUI tidak muncul
```bash
# Fix X11 permissions
xhost +local:

# Update graphics drivers di Windows
# Restart Windows jika perlu
```

### Problem: Font/rendering issues
```bash
sudo apt install fonts-dejavu-core fonts-freefont-ttf
sudo fc-cache -fv
```

## Performance Optimization untuk Robotics

### GPU Acceleration (untuk Gazebo)
```bash
# Check GPU support
glxinfo | grep "direct rendering"
# Should show: direct rendering: Yes

sudo apt install mesa-utils
```

### Environment Variables untuk Better Performance
```bash
# Add to ~/.bashrc
echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
echo 'export PULSE_RUNTIME_PATH=/mnt/wslg/PulseAudio' >> ~/.bashrc
source ~/.bashrc
```

## Final Test - OP3 Simulation dengan GUI

```bash
# Terminal 1: Start simulation
ros2 launch op3_manager op3_simulation.launch.py

# Terminal 2: Start visualization (GUI window should appear)
ros2 launch op3_description op3_display.launch.py
```

**Success indicators:**
- ✅ RViz window opens in Windows desktop
- ✅ OP3 robot model visible dan dapat diinteract
- ✅ Joint states bergerak real-time
- ✅ No "cannot connect to X server" errors

## Requirements Minimum
- **Windows:** Windows 10 Build 19044+ atau Windows 11
- **WSL:** Version 2.0.0+
- **Ubuntu:** 20.04+ (Anda pakai 24.04 ✅)

## Quick Commands Reference
```bash
# Check WSLg status
echo $DISPLAY

# Test GUI
xcalc

# Reset WSL (dari Windows PowerShell)
wsl --shutdown && wsl

# Test OP3 visualization
ros2 launch op3_description op3_display.launch.py
```

---

**Note:** WSLg biasanya sudah aktif default pada WSL2 terbaru. Jika masih error, coba update WSL dan restart Windows.