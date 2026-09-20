#!/usr/bin/bash
# Root-run, demand-controlled side camera manager for the RDK X5.
set -e

export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export HOME=/root

source /opt/tros/humble/setup.bash
source /home/sunrise/risabotcar_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/sunrise/risabotcar_ws/install/risabot_automode/share/risabot_automode/config/disable_shm.xml

# The manager starts no MIPI/VSE pipeline until the dashboard requests a side
# view or validated V4 control declares motion authority.
exec ros2 run risabot_automode side_camera_manager
