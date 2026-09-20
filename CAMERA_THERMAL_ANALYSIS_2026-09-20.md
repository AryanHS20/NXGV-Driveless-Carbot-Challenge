# RDK X5 Camera and Stack Thermal Analysis

**Test date:** 2026-09-20  
**Board:** D-Robotics RDK X5 (`risabot1`)  
**Primary objective:** Preserve enough perception performance for a fully autonomous V4 car while reducing heat and unnecessary camera load.

## Test configuration

- Forward camera: Orbbec Astra, 320x240, 30 Hz color/depth
- Right camera: IMX219, 480x272 when active
- Left camera: OV5647, 480x272 when active
- Side-camera relay and dashboard delivery capped at 5 FPS
- Side cameras managed by a renewable request lease and powered off when unused
- LiDAR, UWB, dashboard, lane perception, obstacle perception and V4 shadow services remained active
- Test abort threshold: 90 degrees C

These were short, sequential measurements rather than steady-state laboratory tests. Later stages inherited heat from earlier stages, so the results show practical load and temperature direction, not a certified thermal limit.

## Measured results

| Camera configuration | Temperature observed | Total CPU | RAM | Assessment |
|---|---:|---:|---:|---|
| Forward Astra only | 77.4-77.9 degrees C | 43-50% | approximately 1.72 GB | Suitable for normal perception |
| Astra plus one side camera | 78.9-82.2 degrees C | 53-59% | approximately 1.78 GB | Suitable for temporary use |
| All three cameras | 84.4-86.7 degrees C | 59-63% | approximately 1.81 GB | Too hot for continuous operation |
| Side cameras disabled again | Fell toward 83 degrees C | 50-59% | approximately 1.72 GB | Cooling with expected thermal inertia |

The all-three-camera temperature was still rising when the stage ended. Sustained operation could reach or exceed 90 degrees C, especially in a warm enclosure or with weak airflow.

## Competition configuration

1. Keep the forward Astra continuously available at 320x240 and approximately 25-30 FPS.
2. Keep both side cameras powered off during ordinary lane following.
3. Activate only the side camera required for parking, recovery or a specific checkpoint.
4. Keep side-camera relay and dashboard delivery at 5 FPS.
5. Avoid operating both side cameras simultaneously unless an algorithm requires both views.
6. Stop dashboard video encoding when there is no viewer.
7. Preserve the forward-camera rate because reducing steering vision to 5 FPS would add excessive perception and control latency.

The active MIPI driver still captures at its native 30 FPS while a side sensor is powered. The 5 FPS limit applies to downstream relay and dashboard delivery. Powering the sensor off when unused provides the largest saving.

## Recommended thermal governor

Suggested initial thresholds for track validation:

- Below 80 degrees C: normal operation.
- 80-85 degrees C: allow at most one side camera.
- Above 85 degrees C: disable optional side cameras and debug video.
- Approaching 90 degrees C: issue a thermal fault and perform a controlled stop.

These thresholds must be validated during longer stationary and moving tests before competition use. A controlled stop must pass through the existing safety controller rather than directly publishing actuator commands.

## Side-camera behavior verified on the board

- Both side sensors initialized successfully when requested individually.
- The dashboard started the selected side camera when its stream was viewed.
- The camera process stopped after the viewer disappeared and the lease expired.
- No MIPI camera processes remained at idle.
- `mipi_relay.max_hz` was set to 5.0.
- `risabot-cams.service` was active and enabled at boot.
- An idle dashboard no longer publishes repeated `off` messages that override other camera requesters.

The camera-request arbitration correction is in commit:

```text
6d5aaa6 fix: release idle side-camera lease
```

## Autonomous-operation finding

The process set present during the thermal test was not the complete autonomous driving stack. The following driving and mission nodes were absent:

- `auto_driver`
- `cmd_safety_controller`
- `servo_controller`
- `signage_detector`
- Additional mission-control nodes from the full bringup

ROS endpoint inspection showed:

```text
/cmd_vel              publisher count: 0
/cmd_vel_auto         publisher count: 0
/cmd_safety_status    publisher count: 0
/traffic_light_state  publisher count: 0
/signage_valid        publisher count: 0
```

Therefore, the tested board state could perceive and display sensor data but could not drive autonomously. The expected command chain is:

```text
camera and LiDAR
    -> lane and mission perception
    -> V4 planner and follower
    -> command safety controller
    -> /cmd_vel
    -> servo controller
    -> steering and drive motor
```

## Next validation

1. Stop the orphaned partial-stack processes.
2. Start one clean, complete bringup instance.
3. Confirm exactly one instance of every required node.
4. Confirm fresh forward camera, LiDAR, UWB, lane, signage and safety topics.
5. Confirm `/cmd_vel_auto` reaches the safety controller and that `/cmd_vel` remains zero in manual or stopped states.
6. Run a wheels-up control test.
7. Repeat the thermal measurements with the complete autonomous stack, including signage inference, safety control and the servo bridge.
8. Perform supervised low-speed track validation with one side camera activated only at the checkpoint that needs it.

The current camera power management is appropriate for the autonomous goal, but a complete-stack thermal test remains required before calling the system competition-ready.
