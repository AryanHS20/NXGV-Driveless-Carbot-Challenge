# Architecture

## Node Graph (Test Branch)

hi

```mermaid
graph LR
    subgraph Sensors
        CAM[Astra Mini Camera]
        LID[YDLiDAR Tmini Plus]
        JOY[Joystick]
    end

    subgraph Perception
        LFC[line_follower_camera]
        OA[obstacle_avoidance]
        OAC[obstacle_avoidance_camera]
        TLD[traffic_light_detector]
        BGD[boom_gate_detector]
    end

    subgraph Action Modules
        TWF[tunnel_wall_follower]
        OBS[obstruction_avoidance]
        PKG[parking_controller]
    end

    subgraph Control
        AD[auto_driver]
        SC[servo_controller]
    end

    CAM -->|/camera/color/image_raw| LFC
    CAM -->|/camera/color/image_raw| OAC
    CAM -->|/camera/color/image_raw| TLD
    CAM -->|/camera/color/image_raw| PKG
    LID -->|/scan| OA
    LID -->|/scan| BGD
    LID -->|/scan| TWF
    LID -->|/scan| OBS

    LFC -->|/lane_error| AD
    OA -->|/obstacle_front| AD
    OAC -->|/obstacle_camera| AD
    TLD -->|/traffic_light_state| AD
    BGD -->|/boom_gate_open| AD

    TWF -->|/tunnel_detected + /tunnel_cmd_vel| AD
    OBS -->|/obstruction_active + /obstruction_cmd_vel| AD
    PKG -->|/parking_complete + /parking_cmd_vel| AD

    JOY -->|/joy| SC
    SC -->|/auto_mode| AD
    SC -->|/set_challenge| AD
    SC -->|/cmd_vel| AD

    AD -->|/cmd_vel| ROBOT[Motors]
    SC -->|/odom| ODOM[Odometry]
```

## Data Flow

```
Joystick → servo_controller → /auto_mode → auto_driver
                                              ↓
                                    State Machine (with lap tracking):
                                    ├─ LANE_FOLLOW → /lane_error → forward + steering
                                    ├─ OBSTRUCTION → /obstruction_cmd_vel
                                    ├─ TUNNEL → /tunnel_cmd_vel
                                    ├─ PARKING → /parking_cmd_vel
                                    ├─ DRIVE_TO_PERP → lane follow to perp area
                                    ├─ TRAFFIC_LIGHT → stop on red/yellow
                                    ├─ BOOM_GATE_1 → open lap 1, closed lap 2
                                    └─ BOOM_GATE_2 → random, stop if closed
                                              ↓
                                          /cmd_vel → Motors
```

## Package Map

```
RISA-bot/src/
├── risabot_automode/          ← Main package (both branches)
│   ├── risabot_automode/
│   │   ├── auto_driver.py         ← Brain: state machine + serial/odometry
│   │   ├── line_follower_camera.py ← Lane error from camera
│   │   ├── traffic_light_detector.py  ← [test only] HSV circle detection
│   │   ├── boom_gate_detector.py      ← [test only] LiDAR barrier detection
│   │   ├── tunnel_wall_follower.py    ← [test only] PD wall following
│   │   ├── obstruction_avoidance.py   ← [test only] Lateral dodge
│   │   └── parking_controller.py      ← [test only] Parallel/perp parking
│   ├── launch/
│   │   ├── bringup.launch.py     ← Main branch launcher
│   │   └── competition.launch.py ← Test branch launcher (all nodes)
│   └── config/
│       └── ydlidar.yaml
├── control_servo/             ← Joystick + hardware interface
│   └── control_servo/
│       └── servo_controller.py   ← Mode toggle, manual driving, hardware odometry
├── obstacle_avoidance/        ← LiDAR front obstacle detection
├── obstacle_avoidance_camera/ ← Camera obstacle detection
├── ros2_astra_camera/         ← Camera driver (third-party)
└── ydlidar_ros2_driver/       ← LiDAR driver (third-party)
```
