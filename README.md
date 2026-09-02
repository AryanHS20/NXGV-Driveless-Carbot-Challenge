# RISA-bot

ROS 2 Intelligent System Autonomy — Competition Robot

## Branches

| Branch          | Purpose                                          | Launch Command                                       |
| --------------- | ------------------------------------------------ | ---------------------------------------------------- |
| `main`          | Individual node testing & development            | `run_risabot`                                        |
| `test`          | Competition mode — all 9 challenges (monolithic) | `ros2 launch risabot_automode competition.launch.py` |
| `refactor-test` | Competition mode — refactored with dashboard     | `ros2 launch risabot_automode bringup.launch.py`     |

> **All documentation lives on `main`.** Other branches only contain code.

---

## 🚀 Fresh Installation (From Scratch)

For setting up a **new robot** (RDK X5 / Ubuntu 22.04 / ROS 2 Humble or TROS):

### Prerequisites

- Ubuntu 22.04 with ROS 2 Humble (or Horizon TROS) installed
- Git installed (`sudo apt install git`)

### Steps

```bash
# 1. Create workspace and clone (includes all dependencies)
git clone https://github.com/eemrull/RISA-bot.git ~/risabotcar_ws

# 2. Run the automated installer
cd ~/risabotcar_ws
bash tools/install.sh

# 3. Reboot to apply udev rules and group permissions
sudo reboot

# 4. (Optional) Enable autostart on boot
sudo bash tools/setup_autostart.sh
```

> **Note:** The main installer (`install.sh`) handles everything: udev rules, YDLidar SDK build, rosdep, colcon build, and bashrc setup. If you ever want to _only_ regenerate your aliases and bash functions without rebuilding the workspace, you can run `bash tools/install_bashalias.sh` independently. All third-party dependencies (Astra Camera, YDLidar SDK, YDLidar ROS2 driver) are **included in the repo** — no need to copy anything manually.

---

## Quick Start (Existing Robot)

```bash
# On your PC — push changes
git add . && git commit -m "message" && git push

# On the robot — pull, build, run
ssh risabot
cd ~/risabotcar_ws/src/RISA-bot
git checkout <branch> && git pull
cd ~/risabotcar_ws && cb && sos

# Run (pick one)
run_risabot                                          # main branch
ros2 launch risabot_automode competition.launch.py   # test branch
ros2 launch risabot_automode bringup.launch.py       # refactor-test branch
```

## Build Aliases

| Alias       | When to Use                               |
| ----------- | ----------------------------------------- |
| `cb`        | Normal rebuild after code changes         |
| `cbp <pkg>` | Rebuild only one package (fastest)        |
| `cbc`       | Clean rebuild — use when stuff is broken  |
| `sos`       | Re-source the workspace (after any build) |

---

## ⚡ Speeding up `colcon build`

After your first successful full build on the robot, the C++ packages (like `ros2_astra_camera` and `ydlidar_ros2_driver`) do not need to be rebuilt unless you update their source code. You can tell `colcon` to ignore them to vastly speed up your build times when working on Python nodes.

The `tools/install.sh` script does this automatically. If you ever need to do it manually or revert it:

```bash
# Ignore the packages (fast builds)
touch ~/risabotcar_ws/src/ros2_astra_camera/COLCON_IGNORE
touch ~/risabotcar_ws/src/ydlidar_ros2_driver/COLCON_IGNORE

# Re-enable the packages (slow builds)
rm ~/risabotcar_ws/src/ros2_astra_camera/COLCON_IGNORE
rm ~/risabotcar_ws/src/ydlidar_ros2_driver/COLCON_IGNORE
```

> **Note:** `rosmaster_lib` is installed natively via `pip`/`setup.py` so it does not need to be built by ROS. To prevent it from showing up in your `colcon build` output and wasting time, its `COLCON_IGNORE` is permanently tracked in the Git repository under `tools/rosmaster_lib/`.

---

## 📖 Guide

Detailed documentation is in the [Guide/](Guide/) folder:

| Guide                                                | Description                                              |
| ---------------------------------------------------- | -------------------------------------------------------- |
| [Odometry](Guide/Odometer_Guide.md)                  | Hardware encoder vs software odometry — equations & code |
| [Challenge Breakdown](Guide/challenges_breakdown.md) | Deep dive into each challenge's code, with course layout |
| [Main Branch](Guide/main_branch.md)                  | How `main` works — individual nodes, original controller |
| [Test Branch](Guide/test_branch.md)                  | How `test` works — state machine, all challenges         |
| [Commands Reference](Guide/commands_reference.md)    | All ROS topics, launch files, `ros2 param set` commands  |
| [Tuning Guide](Guide/tuning_guide.md)                | Step-by-step parameter tuning on physical course         |
| [Architecture](Guide/architecture.md)                | Node graphs, data flow, package structure                |

## 🎓 Workshop

ROS 2 learning materials are in the [Workshop/](Workshop/) folder (9 modules from intro to full integration).

---

## 🖥️ Dashboard (`refactor-test` branch)

The `refactor-test` branch includes a real-time web dashboard at `http://<robot_ip>:8080`.

### Features

- **Hybrid Priority Engine** — Current top-level priority, lap, mode (AUTO/MANUAL), and stop reason
- **Camera Feed** — Raw + debug overlays (Lane Lines, Traffic Light, Obstacle)
- **Traffic Light Visualizer** — Animated red/yellow/green indicator
- **Sensor Panel** — LiDAR, Camera, Fused obstacle, Boom Gate, Tunnel, Obstruction status
- **Odometry** — Distance + speed readout
- **Controller Visualizer** — Button map + live analog joystick visualization
- **Parameter Tuning** — Slide-out drawer with Get/Set for all tunable ROS parameters
- **Event Log** — Timestamped state/mode changes and stop/resume events

---

## Troubleshooting

### astra_camera fails with `openni2_redist` missing

The `openni2_redist` folder should already be at `src/ros2_astra_camera/astra_camera/openni2_redist/` since it's tracked in the repo. If somehow missing, rebuild:

```bash
colcon build --symlink-install --packages-select astra_camera
```

### AMENT_PREFIX_PATH warnings after cbc

```bash
unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/humble/setup.bash
```

## Key Rules

1. Only `git checkout/pull` inside your package dir — never from workspace root
2. All documentation changes go on the `main` branch only
