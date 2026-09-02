# Module 3: Putting It Together

## Learning Objectives

By the end of this module, you will:
- Understand how to combine multiple ROS 2 nodes into a working autonomous system
- Write a **launch file** that starts everything in a single command
- Know how to use `TimerAction` to control node startup order
- Learn how to share a central parameter file across all nodes
- Run the full RISA-bot system using `bringup.launch.py` and understand every piece inside it

---

## 1. The Problem: Too Many Terminals!

Over the previous modules, you ran nodes one at a time across multiple terminals:

| Module | What You Ran | Terminal |
|--------|-------------|----------|
| 1 | `ros2 run joy joy_node` | Terminal 1 |
| 1 | `ros2 run my_robot_controller joy_driver` | Terminal 2 |
| 2 | `ros2 launch astra_camera astra_mini.launch.py` | Terminal 3 |
| 2 | `ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node ...` | Terminal 4 |
| 2 | `ros2 run risabot_automode dashboard` | Terminal 5 |

Every piece of the robot needs its own terminal. On competition day with 10+ nodes, this becomes completely unmanageable — opening terminals, sourcing workspaces, remembering the right commands...

The solution is a **launch file**: a single Python script that starts every node in the correct order, with the correct parameters, in one command.

---

## 2. What is a Launch File?

A launch file is a Python script that tells ROS 2: "Start these nodes, in this order, with these settings." Think of it as a conductor's score for an orchestra — it coordinates all the musicians (nodes) so they play together.

```text
Without a launch file:                    With a launch file:

Terminal 1: ros2 launch astra_camera...   Terminal 1: ros2 launch risabot_automode bringup.launch.py
Terminal 2: ros2 run ydlidar...           
Terminal 3: ros2 run risabot... dashboard   ← That's it! One command starts everything.
Terminal 4: ros2 run risabot... line_follower_camera
Terminal 5: ros2 run risabot... auto_driver
Terminal 6: ros2 run joy joy_node
Terminal 7: ros2 run control_servo servo_controller
...
```

You will use launch files in Module 4 to start the camera, the line follower, the auto driver, the joystick, and the dashboard all at once!

---

## 3. Anatomy of a Launch File

Let's look at the structure of a launch file. Open `src/risabot_automode/launch/bringup.launch.py` and follow along.

### 3.1. Imports

Every launch file starts with these imports:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
```

| Import | What It Does |
|--------|-------------|
| `LaunchDescription` | The container that holds all nodes to launch |
| `Node` | Defines a single ROS 2 node to start |
| `TimerAction` | Delays a node's start by N seconds |
| `IncludeLaunchDescription` | Includes another launch file (like the camera's) |
| `get_package_share_directory` | Finds where a package is installed |

### 3.2. The `generate_launch_description()` Function

Every launch file **must** contain this function. ROS 2 calls it when you run `ros2 launch`:

```python
def generate_launch_description():
    # Find installed package paths
    astra_pkg = get_package_share_directory('astra_camera')
    risabot_pkg = get_package_share_directory('risabot_automode')
    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')

    return LaunchDescription([
        # ... list of nodes goes here ...
    ])
```

The `params_file` variable points to a single YAML file (`config/params.yaml`) that contains the settings for **all** nodes. This is how the RISA-bot keeps all tunable parameters in one place — you don't have to edit 10 different files.

### 3.3. Starting a Node

The simplest way to add a node:

```python
Node(
    package='joy',              # Which ROS 2 package?
    executable='joy_node',      # Which executable inside that package?
    name='joy_node',            # Name to give it in the ROS graph
    output='screen',            # Print its log output to your terminal
    parameters=[{               # Pass parameters directly
        'deadzone': 0.12,
        'autorepeat_rate': 20.0,
    }]
),
```

Or load parameters from a shared YAML file:

```python
Node(
    package='risabot_automode',
    executable='line_follower_camera',
    name='line_follower_camera',
    output='screen',
    parameters=[params_file]    # ← Load from params.yaml
),
```

> [!NOTE]
> As you will see in Module 4, the line follower camera has many parameters like `white_threshold`, `n_scanlines`, and `kalman_enabled`. Instead of typing all of those inline, we point to the `params.yaml` file — much cleaner!

### 3.4. Including Another Launch File

Some packages (like the Astra camera) come with their own launch files. Instead of copying all their node definitions into your file, you can include them:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(astra_pkg, 'launch', 'astra_mini.launch.py')
    )
),
```

This is like saying "run their launch file as part of mine." Remember in Module 2, you ran `ros2 launch astra_camera astra_mini.launch.py` in a separate terminal. Now it is included automatically.

### 3.5. Delaying Node Startup with `TimerAction`

Some nodes depend on others being ready first. For example, the `line_follower_camera` from Module 4 needs the camera to be publishing frames before it can start processing images. Without a delay, it would start, see no camera data, and produce errors.

```python
# Wait 3 seconds for the camera to initialize, then start the line follower
TimerAction(period=3.0, actions=[
    Node(
        package='risabot_automode',
        executable='line_follower_camera',
        name='line_follower_camera',
        output='screen',
        parameters=[params_file]
    ),
]),
```

The `auto_driver` (the brain from Module 4) waits even longer — 5 seconds — because it needs ALL sensors and perception nodes ready before it starts making decisions:

```python
# Wait 5 seconds so all perception nodes are publishing
TimerAction(period=5.0, actions=[
    Node(
        package='risabot_automode',
        executable='auto_driver',
        name='auto_driver',
        output='screen',
        parameters=[params_file]
    ),
]),
```

---

## 4. The RISA-bot Bringup Launch File — Full Walkthrough

Here is the complete startup sequence of `bringup.launch.py`. It brings together all the nodes you have been using across the workshop:

```text
TIME   NODE                      PURPOSE
─────────────────────────────────────────────────────────────────
0s     Astra Camera              Camera driver (Module 2)
0s     YDLiDAR driver            LiDAR driver (Module 2)
0s     TF publisher              Coordinate frame link
0s     cmd_safety_controller     Speed limits & emergency stop (Module 4)
0s     joy_node                  Joystick input (Module 1)
0s     servo_controller          Motor & steering hardware (Module 1)
0s     health_monitor            System health watchdog
0s     dashboard                 Web UI at :8080 (Module 2)
─────────────────────────────────────────────────────────────────
3s     obstacle_avoidance        LiDAR obstacle detection
3s     obstacle_avoidance_camera Camera obstacle detection
3s     line_follower_camera      Lane detection (Module 4)
3s     traffic_light_detector    Traffic light detection
3s     tunnel_wall_follower      LiDAR tunnel navigation (Module 5)
─────────────────────────────────────────────────────────────────
5s     auto_driver               The brain — decides what to do (Module 4)
```

**Notice the three startup groups:**
1. **Immediate (0s):** Hardware drivers and infrastructure — these must start first
2. **Delayed 3s:** Perception nodes — wait for sensors to be publishing data
3. **Delayed 5s:** The brain — waits for everything else to be ready

This is the same pattern used in professional robotics and autonomous vehicles: **hardware → perception → decision-making**.

---

## 5. Hands-On: Write Your Own Launch File

Now let's create a launch file that combines the nodes you used across the workshop into a single command.

### Step 1 — Create the launch directory

```bash
mkdir -p ~/student_ws/src/my_robot_controller/launch
```

### Step 2 — Write the launch file

Create `~/student_ws/src/my_robot_controller/launch/my_bringup.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Find the camera package's launch file
    astra_pkg = get_package_share_directory('astra_camera')
    
    # LiDAR serial port (same one you used in Module 2)
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    return LaunchDescription([

        # ==================== SENSORS (0s) ====================
        # These start immediately — hardware needs to be ready first

        # Camera (include the Astra's own launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_pkg, 'launch', 'astra_mini.launch.py')
            )
        ),

        # LiDAR
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': lidar_port,
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'frequency': 10.0,
            }],
        ),

        # Joystick (from Module 1)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Dashboard (from Module 2)
        Node(
            package='risabot_automode',
            executable='dashboard',
            name='dashboard',
            output='screen',
        ),

        # Your joystick driver (from Module 1)
        Node(
            package='my_robot_controller',
            executable='joy_driver',
            name='joy_driver',
            output='screen',
        ),
    ])
```

This launch file starts the camera, LiDAR, joystick, dashboard, and your joystick driver — all the nodes you used in Modules 1 and 2 — with a single command!

### Step 3 — Register the launch file in `setup.py`

For ROS 2 to find your launch file, you need to tell `setup.py` to install it. Edit `~/student_ws/src/my_robot_controller/setup.py`:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    # ... existing fields ...
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADD THIS LINE to install launch files:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # ... rest of setup ...
)
```

> [!IMPORTANT]
> Don't forget the `import os` and `from glob import glob` at the top of `setup.py`!

### Step 4 — Build and run

```bash
cd ~/student_ws
colcon build --packages-select my_robot_controller
source install/setup.bash

# Launch everything with one command!
ros2 launch my_robot_controller my_bringup.launch.py
```

You should see all nodes start up — camera, LiDAR, joystick, dashboard, and your driver. Open the dashboard in your browser at `http://192.168.x.x:8080` to see both the camera feed and LiDAR data appear — all launched from one terminal!

---

## 6. Adding Delays for Dependent Nodes

Right now, all nodes start at the same time. But what if you want to add the line follower from Module 4? It needs the camera to be publishing first. Let's add it with a delay:

```python
        # ==================== PERCEPTION (3s) ====================
        # Wait for sensors to be ready

        TimerAction(period=3.0, actions=[
            Node(
                package='risabot_automode',
                executable='line_follower_camera',
                name='line_follower_camera',
                output='screen',
                parameters=[{
                    'white_threshold': 100,
                    'invert_binary': True,
                    'n_scanlines': 8,
                    'show_debug': True,
                }]
            ),
        ]),
```

Now the line follower waits 3 seconds for the camera to start. Your launch file is beginning to look like the real `bringup.launch.py`!

---

## 7. Using a Shared Parameter File

In Module 4, you will see that the RISA-bot has many tunable parameters (PID gains, thresholds, speeds). Typing them all inline in the launch file would be messy. Instead, the RISA-bot uses a single `config/params.yaml` file:

```yaml
# config/params.yaml — one file for all nodes
line_follower_camera:
  ros__parameters:
    white_threshold: 100
    invert_binary: true
    n_scanlines: 8
    show_debug: true

auto_driver:
  ros__parameters:
    forward_speed: 0.15
    pid_kp: 0.8
    pid_kd: 0.20
```

Then in the launch file, every node loads from the same file:

```python
risabot_pkg = get_package_share_directory('risabot_automode')
params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')

# Each node points to the shared file:
Node(
    package='risabot_automode',
    executable='line_follower_camera',
    name='line_follower_camera',
    parameters=[params_file]      # ← All params loaded from one place
),
```

> [!TIP]
> This is why you can tune parameters using `ros2 param set` in Module 4 and then click **💾 Save Current as Default** on the dashboard — it writes back to this single `params.yaml` file!

---

## 8. Running the Full RISA-bot System

Now that you understand how launch files work, let's run the real `bringup.launch.py`:

```bash
cd ~/risabotcar_ws
source install/setup.bash
ros2 launch risabot_automode bringup.launch.py
```

Watch for these key messages in the terminal output:

```
[ydlidar_ros2_driver_node]: Now lidar is scanning...
[servo_controller]: ✅ Rosmaster Connected (V9 Competition)
[dashboard]: Dashboard live!
[dashboard]:   → http://10.118.151.222:8080
[line_follower_camera]: Line Follower Camera: Ready
[auto_driver]: Auto Driver Node Starting (Competition Mode)...
[auto_driver]: State: MANUAL
```

Once `auto_driver` prints `State: MANUAL`, the full system is running. Open the dashboard in your browser to verify everything is connected.

### Compare: Your Launch File vs RISA-bot's

| Feature | Your `my_bringup.launch.py` | RISA-bot's `bringup.launch.py` |
|---------|---------------------------|-------------------------------|
| Nodes | ~5 (sensors, joy, dashboard) | 13+ (full competition system) |
| Parameters | Inline values | Central `params.yaml` file |
| Delays | None yet | 3s for perception, 5s for brain |
| Safety | None | `cmd_safety_controller` enforces limits |
| Monitoring | Dashboard only | `dashboard` + `health_monitor` |

The core pattern is identical: **start hardware → wait → start perception → wait → start the brain**. The RISA-bot just has more nodes.

---

## 9. Exercises

### Exercise 1: Add a Startup Delay

Modify your `my_bringup.launch.py` to delay your `joy_driver` node by 2 seconds. This ensures the `joy_node` has time to connect to the gamepad before your driver tries to subscribe.

```python
TimerAction(period=2.0, actions=[
    Node(
        package='my_robot_controller',
        executable='joy_driver',
        name='joy_driver',
        output='screen',
    ),
]),
```

Rebuild and launch. Does the startup feel smoother?

### Exercise 2: Add the Line Follower and Auto Driver

Expand your launch file to include the full lane-following pipeline from Module 4:

1. Add `line_follower_camera` with a 3-second delay
2. Add `cmd_safety_controller` at 0 seconds
3. Add `servo_controller` at 0 seconds
4. Add `auto_driver` with a 5-second delay

Your launch file should now start the complete lane-following system in one command — just like `lane_test.launch.py` from Module 4!

### Exercise 3: Create Your Own Parameter File

Create a `config/params.yaml` file inside your package:

```bash
mkdir -p ~/student_ws/src/my_robot_controller/config
```

Write a simple parameter file:
```yaml
# ~/student_ws/src/my_robot_controller/config/params.yaml
joy_driver:
  ros__parameters:
    max_speed: 0.3
    servo_center: 102
```

Register the config folder in `setup.py`:
```python
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
```

Update your launch file to load from this file instead of inline parameters. Rebuild and verify the parameters are loaded correctly using:
```bash
ros2 param get /joy_driver max_speed
```

---

## 10. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `[ERROR] Package 'xyz' not found` | Package not built or not sourced | Run `colcon build` and `source install/setup.bash` |
| Node starts but no data flows | Started too early, sensors not ready | Add `TimerAction` delay |
| `Address already in use` (dashboard) | Old dashboard process still running | `pkill -f dashboard` then relaunch |
| Launch file not found | Not registered in `setup.py` | Add the `data_files` line for `launch/` |
| Parameters not loading from YAML | File not installed | Add `config/` to `data_files` in `setup.py` |

---

## 11. What You've Learned

In this module you brought everything together:

```text
Individual nodes  →  Launch file  →  One-command startup  →  Full autonomous system
```

You learned that:
- A **launch file** is a Python script that starts multiple nodes in one command
- **`TimerAction`** delays nodes so dependencies (like sensors) start first
- **`IncludeLaunchDescription`** lets you nest other packages' launch files inside yours
- A **shared parameter file** (`params.yaml`) keeps all settings in one place
- The RISA-bot's `bringup.launch.py` uses the exact same patterns you just learned — just with more nodes
- The startup order follows a clear pattern: **hardware → perception → brain**

---

**Previous:** [Module 2 — Dashboard & Sensors](02-dashboard-and-sensors.md)
**Next:** [Module 4 — Lane Following](04-lane-follower.md)
