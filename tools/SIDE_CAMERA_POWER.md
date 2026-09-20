# Demand-controlled side cameras

The Astra forward camera remains continuously available to the lane and sign
pipelines. The IMX219 right-back and OV5647 left-back MIPI/VSE pipelines start
only while requested. Their relayed ROS streams and dashboard JPEG encoding are
capped at 5 Hz; the active MIPI sensor uses 480x272.

The dashboard renews a request while a side-camera viewer is connected and
requests `off` when the viewer returns to Forward or disconnects. Validated V4
motion authority independently keeps the right-back camera available. Driver
failure is latched instead of repeatedly restarting a hot pipeline.

## Install on the RDK X5

```bash
cd /home/sunrise/risabotcar_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select risabot_automode
source install/setup.bash
sudo bash tools/install_side_camera_manager.sh
```

## Inspect

```bash
export ROS_DOMAIN_ID=1
source /opt/tros/humble/setup.bash
source /home/sunrise/risabotcar_ws/install/setup.bash
ros2 topic echo /side_camera/status
```

For calibration, keep a mode leased from a dedicated terminal:

```bash
ros2 topic pub -r 0.5 /side_camera/request std_msgs/msg/String "{data: both}"
```

Stop that publisher with Ctrl+C; both cameras shut down after eight seconds.
