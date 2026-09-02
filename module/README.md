# RISA-bot Educational Modules

This directory contains a series of step-by-step educational guides and documentation for developers and students working with the RISA-bot platform.

## Contents

| Module | Document | Description |
|---|---|---|
| **Module 1** | [ROS 2 Basics & Workflow](01_ros2_basics.md) | Introduction to ROS 2 concepts (nodes, topics, messages, workspaces, packages), workflow commands, and a complete code example of a Python Publisher & Subscriber. |
| **Module 2** | [RISA-bot Controller](02_controller.md) | Details on the Yahboom Rosmaster expansion board, serial communication, motor control limitations (single driven motor), Ackermann steering kinematics, and onboard IMU calibration. |
| **Module 3** | [Camera Operations](03_camera.md) | How to launch the Astra Mini camera, verify topics, view the feed using web/graphical tools, and write an image processing node using `cv_bridge` and OpenCV. |
| **Module 4** | [LiDAR Operations](04_lidar.md) | Running the YDLiDAR driver with stable `by-id` ports, understanding `LaserScan` data structures, adjusting the `3.1416` backward mounting offset, and parsing scans in Python. |
| **Module 5** | [BPU AI Detection](05_ai_detection.md) | BPU hardware acceleration overview, step-by-step custom model conversion pipeline (YOLOv5 -> ONNX -> `.bin` via Horizon toolchain), and NV12 image formatting in Python. |
| **Module 6** | [RISA-bot Features](06_risabot_features.md) | User guide for the built-in web dashboard, joystick game controller mappings, autonomous challenge state machine, and the manual movement record-and-playback system. |
| **Module 7** | [Developer Troubleshooting](07_troubleshooting.md) | Developer notes and solutions compiled from physical testing (USB conflicts, camera wrappers, steering biases, networking hurdles, NoMachine, and RAM limits). |

---

*Note: All modules contain planned image placeholders labeled with `<!-- Image Description: ... -->` comments so you can easily drop in screenshots when uploading to GitHub.*
