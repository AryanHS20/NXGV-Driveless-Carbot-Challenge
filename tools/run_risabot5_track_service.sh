#!/usr/bin/env bash
set -eo pipefail

for _ in $(seq 1 60); do
  if [[ -e /dev/input/js0 ]] \
      && [[ -e /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 ]] \
      && [[ -f /home/sunrise/risabot5_profiles/camera_profiles.yaml ]]; then
    break
  fi
  sleep 1
done

[[ -e /dev/input/js0 ]]
[[ -e /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 ]]

source /opt/tros/humble/setup.bash
source /home/sunrise/risabotcar_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

exec ros2 launch risabot_v4_control track_test.launch.py \
  vehicle:=risabot5 \
  motor_duty:=65 \
  steering_gain:=2.8 \
  minimum_turn_duty:=48 \
  steering_slowdown_gain:=0.85 \
  enable_reverse_recovery:=false \
  profile_path:=/home/sunrise/risabot5_profiles/camera_profiles.yaml \
  dashboard:=true
