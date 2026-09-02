# Module 7: Developer Troubleshooting Guide

This guide compiles known development issues, limitations, and debugging steps collected from physical testing of RISA-bot on the competition track.

---

## 1. Hardware & Port Assignation

### 1.1. USB Port Hardcoding Conflict
- **Symptom**: The LiDAR driver or motor controller fails to open, reporting serial connection errors.
- **Cause**: Linux assigns serial ports (`/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.) dynamically on boot depending on which device powers up first. If hardcoded, the node might try to connect to the wrong physical device.
- **Solution**: Configure udev rules inside the robot's Linux filesystem (`/etc/udev/rules.d/`) to map vendor/product IDs to unique, persistent symlinks (e.g. `/dev/myserial` for the board and `/dev/serial/by-id/...` for the LiDAR).

### 1.2. LiDAR USB-to-UART Silicon Labs Driver
- **Symptom**: The LiDAR USB adapter is plugged into the RDK X5, but `/dev/ttyUSBX` does not appear.
- **Solution**: Install the universal Silicon Labs CP210x USB-to-UART bridge driver for Linux:
  ```bash
  # Check if device is detected
  lsusb | grep "Silicon Labs"
  # If missing, build and insert the cp210x kernel module or install via package manager
  ```

---

## 2. Sensor & Camera Troubleshooting

### 2.1. Camera Wrapper (Orbbec Astra Mini)
- **Symptom**: Launching camera nodes results in compilation or openni connection failures.
- **Solution**: Ensure you run the **`astra_mini.launch.py`** launcher file in the `astra_camera` package rather than generic Orbbec wrappers. The Astra Mini uses specific OpenNI2 configurations.
  ```bash
  ros2 launch astra_camera astra_mini.launch.py
  ```

### 2.2. Camera Memory Leak & Crash
- **Symptom**: Nodes crash randomly with `Out of Memory` errors or the system freezes.
- **Cause**: Processing full-resolution frames takes significant CPU and memory.
- **Solution**: Limit RAM usage on the RDK X5 by avoiding running multiple intensive graphical nodes concurrently. If the camera crashes, perform serial/parallel work sequentially, crop ROIs early, or decrease the processing resolution to `320x240` in the computer vision pipelines.

---

## 3. Autonomous Driving & Calibration

### 3.1. Ackerman Steering Asymmetry Bias
- **Symptom**: The robot steers correctly to the left but turns very slowly/weakly to the right.
- **Cause**: Structural and mechanical tolerances in the Ackermann steering linkage.
- **Solution**: Calibrate the center and range parameters in `servo_controller.py`. Use the `auto_right_steer_boost` parameter (set to `1.3` or higher) to amplify servo commands when steering right to match left-turn performance.
  ```bash
  ros2 param set /servo_controller auto_right_steer_boost 1.4
  ```

### 3.2. Odometry Accuracy
- **Symptom**: Distance or position reported on the dashboard drifts significantly.
- **Cause**: Motor encoders are only mounted on one wheel, causing slip errors, and Ackerman geometry requires careful calibration of `ticks_per_meter`.
- **Solution**: Follow the calibration steps in the [Odometry Guide](../Guide/Odometer_Guide.md). Adjust `ticks_per_meter` or `odom_distance_scale` and `odom_yaw_scale` to match actual physical displacements.

### 3.3. IMU Integration Difficulty
- **Symptom**: Raw IMU data is readable, but integrating it for yaw tracking causes wild drift.
- **Solution**: Zero the offsets using the dashboard calibration tool. When stationary, the system filters out minor values (under 0.25°) via deadbands to prevent drift.

---

## 4. AI & BPU Compilation

### 4.1. Supported Model Types
- **Symptom**: The Horizon compilation toolchain outputs errors during ONNX parsing.
- **Cause**: You are attempting to compile a model architecture not supported by the hardware BPU.
- **Solution**: **Use YOLOv5 strictly.** The Horizon toolchain is pre-configured and optimized to process YOLOv5 layers. Avoid YOLOv8, YOLOv10, or custom model variants unless you write custom layer mapping plugins.
- **Important**: The RDK X5 only supports `.bin` compiled model formats. Standard `.pt` (PyTorch) or `.onnx` files are not BPU-compatible.

---

## 5. Networking & Interface Issues

### 5.1. Dashboard Shows "Disconnected" State
- **Symptom**: Webpage loads, but fields show `—` and the status reads `Disconnected`.
- **Cause**: The background HTTP handler thread is blocked or timing out, or the network connection has dropped.
- **Solution**: Restart the dashboard node:
  ```bash
  ros2 run risabot_automode dashboard
  ```
  Ensure your laptop is on the same local Wi-Fi subnet as the robot.

### 5.2. SSH Blocked on Public Networks
- **Symptom**: Cannot ping or SSH into the robot's IP.
- **Cause**: Enterprise and university Wi-Fi networks (like UIA) block peer-to-peer communication between devices on the network.
- **Solution**: Connect both the robot and your laptop to a local mobile hotspot.

### 5.3. Remote Desktop Access (NoMachine)
- **Symptom**: Running graphical tools over SSH is slow or fails.
- **Solution**: Install **NoMachine** on both the RDK X5 and your laptop. It is significantly faster and more resource-efficient than VNC, allowing you to debug graphical applications without an HDMI monitor plugged into the robot.

![NoMachine graphical connection interface](images/nomachine_connection.png)
<!-- Image Description: Screenshot of NoMachine interface showing a high-speed remote desktop session connected to the RDK X5 Ubuntu desktop. -->
