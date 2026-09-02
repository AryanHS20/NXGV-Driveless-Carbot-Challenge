# RISA-bot Workshop

Hands-on workshop modules for learning ROS 2 using the RISA-bot platform.

## Prerequisites

- Ubuntu 22.04 (on robot or VM)
- ROS 2 Humble installed
- SSH access to the robot (`ssh risabot`)
- Basic Python and Linux terminal knowledge

## Modules

| #   | Topic                                                        | Duration | Description                                        |
| --- | ------------------------------------------------------------ | -------- | -------------------------------------------------- |
| 0   | [Linux Basics](00-linux-basics.md)                           | 30 min   | Terminal commands, SSH, file navigation             |
| 1   | [Introduction to ROS 2](01-introduction-to-ros.md)           | 120 min  | Setup, Nodes, Topics, & Joystick                   |
| 2   | [Dashboard & Sensors](02-dashboard-and-sensors.md)           | 60 min   | Headless robot monitor, Camera & LiDAR data        |
| 3   | [Putting It Together](03-putting-it-together.md)             | 60 min   | Combining nodes, launch files & shared params      |
| 4   | [Lane Following](04-lane-follower.md)                        | 60 min   | Image pipeline, PID control, launch & tuning       |
| 5   | [Tunnel Navigation](05-tunnel-navigation.md)                 | 60 min   | LiDAR wall following, RANSAC, PD control           |
| 6   | [Autonomous Parking](06-autonomous-parking.md)               | 60 min   | Recorded trajectories, signage triggers, playback  |
| 7   | [Computer Vision](07-computer-vision.md)                     | 60 min   | HSV thresholding, BPU inference, CV boom gate      |

## How to Use

- Each module is **self-contained** — pick the ones relevant to your workshop
- **Module 0** is recommended for students new to Linux terminals
- Modules build on each other sequentially, but you can skip ahead
- All exercises use the RISA-bot hardware and codebase
- Code examples reference actual files in this repository

## Setup Before Workshop

```bash
# On the robot
cd ~/risabotcar_ws
git checkout main && git pull
cb
sos
```
