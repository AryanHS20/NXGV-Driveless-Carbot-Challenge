# RISA-bot New Robot Deployment Guide

> **Time estimate:** ~20–30 min per robot (mostly waiting for the build).  
> **Prerequisite:** Robot is on the same network, SSH is reachable, ROS 2 Humble is pre-installed on the OS image.

---

## Step 0 — Identify the Robot IP

Power on the robot and find its IP address (check your router, or connect a monitor):

```bash
# From your local machine, test connectivity
ping <ROBOT_IP>
ssh sunrise@<ROBOT_IP>     # default password: sunrise
```

---

## Step 1 — Clone the Repository

```bash
# On the ROBOT (via SSH)
cd ~
git clone https://github.com/eemrull/RISA-bot.git risabotcar_ws
cd risabotcar_ws
git checkout refactor-test
git lfs install
git lfs pull
```

---

## Step 2 — Run the Deployment Script

This installs all system dependencies, builds the workspace, and sets up udev rules and environment variables automatically.

```bash
cd ~/risabotcar_ws
bash tools/install_deps.sh
```

> ⚠️ This will take **10–20 minutes**. The script will:
> - Install ROS packages (`joy`, `opencv`, `numpy`, build tools)
> - Build YDLidar SDK, libuvc, magic_enum from source
> - Install Rosmaster_Lib
> - Install Astra camera USB rules
> - **Write the correct udev rules** for the motor board and LiDAR
> - Build the full workspace with `colcon build`
> - Configure `~/.bashrc` with workspace source and DDS fix

---

## Step 3 — Apply the Correct Hardware Udev Rules ⚠️ CRITICAL

Even though the install script writes the rules, you must **physically replug the USB devices** for the symlinks to activate.

**Unplug and replug BOTH USB cables:**
- Motor board USB (CH340 chip — black cable to Rosmaster board)
- LiDAR USB (Silicon Labs CP2102 — thin cable to Tmini Plus)

Then verify the mappings are correct:

```bash
ls -l /dev/myserial   # MUST point to ttyUSB0  ← Motor board
ls -l /dev/ydlidar    # MUST point to ttyUSB1  ← LiDAR
```

**If `/dev/myserial` still points to `ttyUSB1`**, the rules file may be stale. Fix it manually:

```bash
sudo tee /etc/udev/rules.d/99-risabot.rules > /dev/null << 'EOF'
# RISA-bot UDEV Rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="myserial"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug the motor board USB cable** again.

**Quick sanity check — verify USB hardware IDs:**

```bash
# Motor board should be 1a86:7523
udevadm info /dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"

# LiDAR should be 10c4:ea60
udevadm info /dev/ttyUSB1 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"
```

---

## Step 4 — Source the Environment

```bash
source ~/.bashrc
```

Or open a fresh SSH session (recommended — all env vars will be clean).

---

## Step 5 — Launch the Stack

For the full autonomous stack, launch the competition stack:

```bash
ros2 launch risabot_automode competition.launch.py
```

Wait ~10s for all nodes to appear. Verify with:

```bash
ros2 node list
```

Expected nodes:
```
/astra_camera_container
/auto_driver
/base_to_laser
/camera/camera
/cmd_safety_controller
/health_monitor
/joy_node
/servo_controller
/ydlidar_ros2_driver_node
/obstacle_avoidance_node
/obstacle_avoidance_camera
/line_follower_camera
/traffic_light_detector
/boom_gate_detector
/tunnel_wall_follower
/obstruction_avoidance
/parking_controller
/signage_detector
/dashboard
```

---

## Step 5.5 — Copy and Verify the YOLOv5 BPU Model

Since the RISA-bot now uses BPU-accelerated signage, hill, and traffic light detection, you must deploy the compiled model file to the robot.

1. **Copy the compiled BPU model file from your local developer PC to the robot:**
   ```bash
   scp tools/bpu_model/model_output/risabot_signs_640x640_nv12.bin sunrise@<ROBOT_IP>:/home/sunrise/
   ```
2. **On the Robot (via SSH), verify BPU hardware execution and model load:**
   ```bash
   cd ~/risabotcar_ws
   python3 tools/bpu_model/verify_bpu.py
   ```
   *Expected output:* `SUCCESS: BPU model loaded successfully.` and `Verification Successful! BPU model is fully functional.`

3. **Verify live camera detection processing (with the robot stationary):**
   ```bash
   python3 tools/bpu_model/verify_live.py
   ```
   *Expected output:* Successfully receives frames and processes BPU inference, outputting diagnostic detection scores for parking signs, hill signs, and traffic lights.

---

## Step 6 — Unlock the RC Controller

The software has a **safety lock** that prevents the robot from moving on startup (prevents ghost inputs from joystick drift). You must do this **every time you launch**:

1. **Connect the joystick** via USB before launching.
2. **Press any button** (e.g. `A`) on the controller.
   - Terminal should log: `Controller unlocked (Button press detected)`
3. **Return both thumbsticks to dead center.**
   - Terminal should log: `Controller neutral detected, manual drive enabled`
4. The robot is now drivable in RC mode.

> If the robot does not move after this sequence, check the launch terminal for `❌ Failed to connect to Rosmaster`. This means `/dev/myserial` is still wrong — go back to **Step 3**.

---

## Step 7 — Verify Everything Is Working

| Check | Command | Expected |
|---|---|---|
| All nodes running | `ros2 node list` | 10+ nodes |
| LiDAR publishing | `ros2 topic hz /scan` | ~10 Hz |
| Camera publishing | `ros2 topic hz /camera/color/image_raw` | ~15–30 Hz |
| Joy reading | `ros2 topic echo /joy --once` | axes and buttons arrays |
| Motor port | `ls -l /dev/myserial` | `-> ttyUSB0` |
| Dashboard | Open `http://<ROBOT_IP>:8080` | Web UI visible |

---

## Troubleshooting Reference

### `servo_controller` dies (exit code 1)
**Cause:** `/dev/myserial` points to the LiDAR port. The LiDAR driver already holds that port open, so the motor driver crashes with a "device busy" error.  
**Fix:** Redo Step 3. Verify `ls -l /dev/myserial` shows `ttyUSB0`.

### `package 'joy' not found` at launch
**Fix:** `sudo apt install -y ros-humble-joy` then re-source.

### Camera: `wait for device connect...`
**Fix:** Unplug and replug the camera USB. The Orbbec Astra needs the udev permission rules **and** a physical reconnect to activate.

### LiDAR doesn't scan / node crashes
**Fix:** Verify the LiDAR port in `bringup.launch.py` matches `ls -l /dev/serial/by-id/`. Should be the Silicon Labs `CP2102` entry.

### Multiple nodes crash on fresh install (exit code 1)
**Cause:** Python dependencies missing (`cv2`, `numpy`, `Rosmaster_Lib`).  
**Fix:** Re-run `bash tools/install_deps.sh`.

### `/dev/myserial` still points to `ttyUSB1` after udev trigger
**Cause:** `udevadm trigger` does not remove symlinks created by old rules for already-connected devices.  
**Fix:** Write rules inline with `tee` (see Step 3 manual fix), **then physically replug** the motor board USB cable.

---

## USB Device Reference

| Device | Chip | Vendor:Product | Symlink |
|---|---|---|---|
| Rosmaster Motor Board | CH340 | `1a86:7523` | `/dev/myserial` |
| YDLiDAR Tmini Plus | Silicon Labs CP2102 | `10c4:ea60` | `/dev/ydlidar` |
| Orbbec Astra Camera | — | via USB rules | Handled by `56-orbbec-usb.rules` |
