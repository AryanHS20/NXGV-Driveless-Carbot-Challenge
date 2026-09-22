#!/usr/bin/env bash
set -eo pipefail

profile=/home/sunrise/risabot1_track_ws/install/risabot_v4_experimental/share/risabot_v4_experimental/config/camera_profiles.yaml
lidar=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

for _ in $(seq 1 60); do
  if [[ -e /dev/input/js0 && -e "$lidar" && -f "$profile" ]]; then
    break
  fi
  sleep 1
done

[[ -e /dev/input/js0 ]]
[[ -e "$lidar" ]]
[[ -f "$profile" ]]

source /opt/tros/humble/setup.bash
source /home/sunrise/risabotcar_ws/install/setup.bash
source /home/sunrise/risabot1_track_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

exec ros2 launch risabot_v4_control track_test.launch.py \
  vehicle:=risabot1 \
  motor_duty:=65 \
  steering_gain:=2.8 \
  minimum_turn_duty:=48 \
  steering_slowdown_gain:=0.85 \
  enable_reverse_recovery:=false \
  profile_path:="$profile" \
  dashboard:=true
