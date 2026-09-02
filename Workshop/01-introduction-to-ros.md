# Module 1: Introduction to ROS 2 & Joystick Control

## Learning Objectives

By the end of this module, you will:
- Understand what ROS 2 is and why robots use it.
- Set up a ROS 2 workspace from scratch.
- Understand the core concepts: **Nodes**, **Topics**, and **Messages**.
- Create a custom Python package and write a ROS 2 **Subscriber** node.
- Interface directly with the RISA-bot hardware to drive it using a gamepad joystick.

---

## 1. What is ROS 2?

**ROS** (Robot Operating System) is not an operating system like Windows or Linux. It is a **framework** that helps different robot software components talk to each other.

Think of it like a messaging system or a radio network:
- Each piece of your robot (camera, motor, AI brain, joystick) runs as a **separate program** called a **Node**.
- Nodes communicate by sending **Messages** through named channels called **Topics**.

### Why ROS 2?
| Without ROS | With ROS |
| ----------- | -------- |
| One giant program does everything. | Each function is a separate node. |
| Change one thing, break everything. | Change one node, others keep working. |
| Hard to reuse code across robots. | Packages can be shared and reused. |
| Must write your own communication logic. | Built-in messaging system. |

---

## 2. Connecting via SSH (Windows)

Because the RISA-bot runs "headless" (without a monitor attached), you will do all your programming remotely from your own laptop using a tool called **SSH** (Secure Shell). This lets you type commands on your laptop that execute directly on the robot's computer.

To connect from a Windows laptop:
1. Open PowerShell or Command Prompt.
2. Type the following command (replace `192.168.x.x` with the robot's actual IP address):
   ```bash
   ssh sunrise@192.168.x.x
   ```
3. If it asks "Are you sure you want to continue connecting?", type `yes` and press Enter.
4. When prompted, type the password: `risabot` (the characters will be invisible as you type, this is normal!)

You are now controlling the robot's terminal! Any command you run here runs on the robot.

---

## 3. Installing ROS 2 Humble

If you are using the provided RISA-bot or a pre-configured Ubuntu 22.04 system, ROS 2 might already be installed. If you are starting completely from scratch on Ubuntu 22.04, run these commands in your terminal:

```bash
# 1. Set up locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add the ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# 4. Install Colcon (the build tool)
sudo apt install python3-colcon-common-extensions -y
```

**Crucial Step:** Every time you open a new terminal, ROS 2 needs to be loaded into your environment. You do this by "sourcing" the setup file:

```bash
source /opt/ros/humble/setup.bash
```
*(Tip: Add this line to the bottom of your `~/.bashrc` file so it runs automatically every time you open a terminal!)*

---

## 4. Creating Your Workspace and Package

A **Workspace** is a folder where you build your custom ROS 2 code. A **Package** is a folder inside the workspace that contains your nodes.

Let's create your own isolated workspace, completely separate from the main RISA-bot code.

```bash
# 1. Create a workspace folder with a 'src' directory
mkdir -p ~/student_ws/src
cd ~/student_ws/src

# 2. Create your first Python package
ros2 pkg create --build-type ament_python my_robot_controller
```

Your workspace now looks like this:
```text
student_ws/
└── src/
    └── my_robot_controller/
        ├── my_robot_controller/  ← Your Python scripts go here
        │   └── __init__.py
        ├── package.xml           ← Lists dependencies
        ├── resource/
        │   └── my_robot_controller
        ├── setup.cfg
        ├── setup.py              ← Tells ROS how to install your code
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

---

## 5. The Talker & Subscriber: Joystick Control

We are going to build a system where you can drive the robot using a gamepad. 

### The Talker (Publisher)
Instead of writing a "Talker" from scratch, we will use a pre-existing ROS 2 package called `joy`. ROS 2 comes with many built-in packages so you don't have to reinvent the wheel!

The `joy_node` is a ready-to-use Talker that reads your gamepad hardware. When you run it, it constantly **Publishes** button presses and joystick movements as a `Joy` message on the `/joy` **Topic**.

To see this built-in Talker in action:
1. Plug a gamepad into your computer.
2. Open a terminal and run the Talker:
   ```bash
   ros2 run joy joy_node
   ```
3. Open a *second* terminal and use the `echo` command to "listen" to the topic:
   ```bash
   ros2 topic echo /joy
   ```
If you move the joysticks, you will see a stream of numbers appearing on your screen! That is the Talker broadcasting its data.

**Configuring Your Controller:**
Because different gamepads (Xbox, PlayStation, generic) map their joysticks to different array indexes, you can use this `echo` output to configure your code. 
- While watching the screen, push your **Right Joystick Left and Right**. 
- Look at the `axes: [0.0, 0.0, 0.0, ...]` array on your screen. 
- Whichever number changes from `0.0` to `1.0` or `-1.0` is your steering axis! (Usually index `2` or `3`). You will use this number in the code later.

### How They Work Together (The rqt_graph)
If you were to look at the ROS 2 communication map (called an `rqt_graph`), you would see this:

```text
[ Talker Node ]  ──publishes──▶  /joy (topic)  ──▶  [ Your Terminal ]
  (joy_node)                   Joystick Data       (_ros2cli_echo_123)
```
*Note: The `echo` command actually creates an invisible Subscriber node just to print the data to your screen!*

### The Subscriber (Your Node)
You will write a Node that **Subscribes** to the `/joy` topic. Whenever you move the joystick, your Node will receive the message, calculate the required speed and steering, and send the physical electrical signals to the RISA-bot's motors.

When you run your node alongside `joy_node`, the map will look like this:

```text
[ Talker Node ]  ──publishes──▶  /joy (topic)  ──▶  [ Subscriber Node ]
  (joy_node)                   Joystick Data         (joy_driver.py)
```

The talker will be a node publishing to a topic, and the subscriber node will hear it. 

**Just imagine:** A talker is someone at YouTube publishing a video. The video channel itself is the **Topic**, and its contents (the actual video) is the **Message** that they want to show or tell their viewers. The viewers are the **Subscribers** who get the information from the video.

Because of this architecture, one talker node can communicate with multiple subscriber nodes simultaneously! In other words, one sensor node (like a camera or joystick) can send its data to multiple processing nodes at the exact same time without them interfering with each other.

---

Create a new file at `~/student_ws/src/my_robot_controller/my_robot_controller/joy_driver.py` and paste the following code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Import the RISA-bot hardware library
from Rosmaster_Lib import Rosmaster

class JoyHardwareDriver(Node):
    def __init__(self):
        super().__init__('joy_hardware_driver')

        # Connect to the hardware board
        try:
            self.bot = Rosmaster()
            self.get_logger().info("Connected to Rosmaster hardware!")
        except Exception as e:
            self.get_logger().error(f"Hardware connection failed: {e}")
            return

        # Create a Subscriber that listens to the /joy topic
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Settings
        self.max_speed = 0.3    # Max PWM speed modifier
        
        # Steering Calibration (just like the real RISA-bot!)
        self.servo_center = 102
        self.servo_range_left = 50
        self.servo_range_right = 70

        self.get_logger().info("Joystick Driver Started! Waiting for input...")

    def joy_callback(self, msg):
        """This function runs every time a message is received on /joy"""
        
        # Extract axis values from the joystick (values are between -1.0 and +1.0)
        # Note: Axis mapping can vary by controller.
        throttle_raw = msg.axes[1]  # Left Stick Up/Down
        steer_raw = msg.axes[3]     # Right Stick Left/Right (Axis 3 for your controller)

        # 1. Calculate Motor Speed (PWM: -255 to +255)
        # Positive PWM goes forward, negative goes backward
        motor_pwm = int(throttle_raw * self.max_speed * 255)

        # 2. Calculate Steering Angle (Servo: 40 to 140 degrees)
        # We use asymmetric left/right ranges to compensate for physical mechanical biases
        if steer_raw >= 0:
            steer_angle = int(self.servo_center - (steer_raw * self.servo_range_left))
        else:
            steer_angle = int(self.servo_center - (steer_raw * self.servo_range_right))
        
        # Clamp the angle so we don't break the physical steering mechanism
        min_angle = self.servo_center - self.servo_range_left
        max_angle = self.servo_center + self.servo_range_right
        steer_angle = max(min_angle, min(max_angle, steer_angle))

        # 3. Send commands to the physical hardware!
        # set_motor(motor1, motor2, motor3, motor4) - RISA-bot uses motor 1 for main drive
        self.bot.set_motor(motor_pwm, 0, 0, 0)
        
        # set_pwm_servo(servo_id, angle) - RISA-bot steering is on servo port 4
        self.bot.set_pwm_servo(4, steer_angle)

        # Print to the terminal
        self.get_logger().info(f"Drive: PWM={motor_pwm} | Steer={steer_angle}", throttle_duration_sec=0.5)


def main(args=None):
    # Initialize the ROS 2 communication system
    rclpy.init(args=args)
    
    # Create the node
    node = JoyHardwareDriver()
    
    # Spin keeps the node running, listening for messages
    rclpy.spin(node)
    
    # Clean up and stop motors when shutting down
    node.bot.set_motor(0, 0, 0, 0)
    node.bot.set_pwm_servo(4, node.servo_center)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Registering Your Node
ROS 2 needs to know your script exists. Edit `~/student_ws/src/my_robot_controller/setup.py`. Find the `entry_points` section at the bottom and update it:

```python
    entry_points={
        'console_scripts': [
            'joy_driver = my_robot_controller.joy_driver:main',
        ],
    },
```

Edit `~/student_ws/src/my_robot_controller/package.xml` to declare your dependencies. Add these lines before `</package>`:
```xml
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
```

---

## 6. Building and Driving

Now we compile your workspace. Any time you change `setup.py` or create a new node, you must build!

```bash
cd ~/student_ws
colcon build --packages-select my_robot_controller
```

After building, you must "source" your workspace so ROS 2 can find your new package:
```bash
source ~/student_ws/install/setup.bash
```

### Let's Drive!
Ensure your gamepad is connected to the robot. You will need **two terminals**.

**Terminal 1 (The Talker):**
```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node
```

**Terminal 2 (Your Subscriber):**
```bash
source /opt/ros/humble/setup.bash
source ~/student_ws/install/setup.bash
ros2 run my_robot_controller joy_driver
```

Push the left stick up/down to drive, and the right stick left/right to steer. Congratulations! You've written your first ROS 2 node and integrated it with physical hardware!

---
**Previous:** [Module 0 — Linux Basics](00-linux-basics.md)
**Next:** [Module 2 — Dashboard & Sensors](02-dashboard-and-sensors.md) to learn how to use the dashboard and read data from cameras and LiDAR.
