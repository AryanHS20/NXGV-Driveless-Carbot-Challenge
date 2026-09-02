# Module 1: ROS 2 Basics & Workflow

Welcome to the ROS 2 (Robot Operating System) module! This guide is designed to familiarize you with the ROS 2 environment, core workflow, commands, and node creation using Python. Understanding these concepts is essential before working on the RISA-bot software.

---

## 1. What is ROS 2?

ROS 2 is a middleware framework that helps developers build robot applications. It handles communication between different software components (called **Nodes**), allowing them to pass messages to one another even if they are running on different hardware or written in different programming languages.

---

## 2. Core Concepts

### 2.1. Nodes
A **Node** is a single process that performs a specific task (e.g., reading LiDAR scans, calculating camera lane errors, or controlling servos). By breaking the robot's brain into small, isolated nodes, the system becomes modular, easier to debug, and robust.

### 2.2. Topics
Nodes communicate with each other by sending messages over a **Topic**. Think of a topic as a public news feed or a channel. 
- A node that sends data is a **Publisher**.
- A node that receives data is a **Subscriber**.

```text
  [Publisher Node] ─── (Publish Message) ───> [ /topic_name ] ─── (Receive Message) ───> [Subscriber Node]
```

![ROS 2 Node and Topic Communication Graph](images/ros2_node_topic_communication.png)
<!-- Image Description: A diagram showing Node A publishing to a Topic, and Node B subscribing to that Topic, with arrows showing data flow direction. -->

### 2.3. Messages
The data sent over a topic is formatted as a **Message** (`.msg`). Messages have strict types (e.g., integer, float, string, or complex sensor structures like `sensor_msgs/msg/LaserScan`).

---

## 3. Workspace Workflow

ROS 2 projects are organized inside a directory called a **Workspace**. On RISA-bot, the main workspace is `~/risabotcar_ws`.

### 3.1. Sourcing ROS 2
Before running any ROS 2 commands, you must source the global ROS 2 environment so your terminal knows where the tools are located:
```bash
source /opt/ros/humble/setup.bash
```

### 3.2. Sourcing Your Workspace
Any time you build packages in your workspace, you must source the local environment so ROS 2 can find your custom packages and nodes:
```bash
cd ~/risabotcar_ws
source install/setup.bash
```

### 3.3. Building the Workspace
We use `colcon` to compile our workspace. If you make changes to Python files, launch configurations, or compile C++ nodes, you need to rebuild:
```bash
# Build a specific package (recommended - much faster)
colcon build --packages-select risabot_automode

# Build everything
colcon build
```

---

## 4. Key CLI Commands

Here are the most common commands you will use to inspect and debug a running ROS 2 system:

| Command | Description | Example |
|---|---|---|
| `ros2 run <pkg> <node>` | Runs a specific node | `ros2 run risabot_automode dashboard` |
| `ros2 launch <pkg> <launch_file>` | Runs a group of nodes via a launch script | `ros2 launch risabot_automode competition.launch.py` |
| `ros2 node list` | Lists all active nodes | `ros2 node list` |
| `ros2 node info <node>` | Shows details (pubs, subs, params) for a node | `ros2 node info /servo_controller` |
| `ros2 topic list` | Lists all active communication channels | `ros2 topic list` |
| `ros2 topic echo <topic>` | Prints real-time messages on a topic | `ros2 topic echo /lane_error` |
| `ros2 topic hz <topic>` | Shows the update rate (frequency) of a topic | `ros2 topic hz /scan` |
| `ros2 param set <node> <param> <val>` | Changes a parameter dynamically at runtime | `ros2 param set /auto_driver pid_kp 0.7` |

---

## 5. Writing a Simple Python Node

Below is a complete walkthrough of a Publisher and a Subscriber node written in Python. This demonstrates the standard structure of a ROS 2 node.

### 5.1. The Publisher Node
This node publishes a simple counter message on the `/hello_topic` topic at a rate of 2 Hz.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        # Initialize node with the name 'simple_publisher'
        super().__init__('simple_publisher')
        
        # Create a publisher on topic '/hello_topic' with message type String.
        # Queue size is 10 (buffers up to 10 messages if subscriber is slow).
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
        # Set a timer to trigger the callback every 0.5 seconds (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World! Count: {self.count}'
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node) # Spin keeps the node running, processing timer events
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2. The Subscriber Node
This node listens to the `/hello_topic` topic and prints any incoming messages to the console.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        # Initialize node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')
        
        # Create a subscription to '/hello_topic' with message type String.
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10 # Queue size
        )

    def listener_callback(self, msg):
        # This function runs automatically whenever a new message arrives
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. How to Run rqt_graph
`rqt_graph` is a graphical tool that displays a live map of all active nodes and topics. It is highly recommended for understanding how nodes connect.

1. Ensure the nodes are running on the robot.
2. In a desktop environment (such as via NoMachine or X11 forwarding), run:
   ```bash
   ros2 run rqt_graph rqt_graph
   ```

![rqt_graph Screenshot showing RISA-bot connections](images/rqt_graph_screenshot.png)
<!-- Image Description: Screenshot of rqt_graph window depicting risabot nodes (like servo_controller, auto_driver, etc.) connected via topic arrows. -->
