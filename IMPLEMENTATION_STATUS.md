# Local implementation status — 15 September 2026

These changes are local, uncommitted, and not deployed or physically calibrated.
The older handoff and architecture files describe the previous deployed system.

## Implemented

### Safety and actuation

- Autonomous driving and recorded parking both request commands through `cmd_safety_controller`. Playback no longer writes motor outputs directly.
- Camera and scan source timestamps, usable scan points, successful signage inference, and command freshness gate motion. A heartbeat does not renew the age of an old observation.
- E-stop and explicit zero commands bypass acceleration ramps. NaN/Inf commands are rejected.
- The safety node checks the actual slew-limited command against a forward/reverse Ackermann footprint sweep. It requires directional scan returns; it does not accept an entirely unobserved travel direction.
- The servo controller checks a fresh motion permit, cancels playback on interlock/watchdog failure, and forces AUTO back to MANUAL after a safety fault. Resume requires deliberate re-arming. Normal red-light waits still resume autonomously on confirmed green.
- Motor duty is bounded to the bundled library's ±100 range; 127 can no longer accidentally invoke “retain previous duty.” The bundled library propagates motor transport errors.

### Perception and driving

- Camera processing loss immediately reports lane lost. Centered observations correct the Kalman estimate.
- Absolute world heading is no longer added to steering. Default `lane_control_mode: direct` consumes normalized lane steering once. `legacy_pid` remains available for a deliberate comparison; it does not restore the heading bug.
- Sliding mode honors `ipm_enabled`. Scanline Otsu honors lane polarity. Defaults seek white paint in scanline mode.
- Metric Pure Pursuit requires explicit `pp_metric_enabled` and IPM. It uses configured ground-plane dimensions in metres; otherwise the image-space controller remains explicitly heuristic.
- Obstruction control uses odometry progress and pose-relative pursuit. Left steering now has the correct sign. Critical clearance, stale observations, and maneuver timeouts stop instead of claiming success. No automatic blind reverse.
- Tunnel centerlines use forward points only, and insufficient geometry stops the car. Current tunnel processing is binned centerline PD, not RANSAC; obsolete RANSAC controls are removed from the dashboard.
- LiDAR consumers share the physical mounting convention. Simulation overrides it to zero.
- Sign confirmation uses separate activation/release hysteresis. Live threshold updates use proposed values. Obstacle signs publish a separate advisory topic instead of overwriting camera obstacle observations.

### Mission and parking

- Stop constraints precede movement selection. Red remains latched through unknown/stale lamp observations until confirmed green.
- Roundabout exit selection uses gate state and a visible lane branch, not traveled distance. An exit is acknowledged after branch confirmation and departure from the observed roundabout sign.
- A closed gate ordinarily stops motion. An observed right branch may divert only with valid rightward lane steering; the final footprint sweep still checks clearance. This is a perception-based implementation requiring track validation, not a proven route planner for every junction geometry.
- Parallel and perpendicular sign types remain distinct. Each parking operation requires a typed recording, start acknowledgement, fresh progress, and explicit completion. Missing recordings, feedback loss, interruption, and timeout are failures.
- Obstacle-sign detections publish on `/obstacle_sign_detected`. The mission stops on a fresh sign advisory until LiDAR confirms and runs the obstruction behavior; the sign publisher is kept separate from physical camera-obstacle sensing.
- Parallel completion resumes driving; perpendicular completion latches FINISHED and commands zero until an explicit mission reset.
- The separate parking controller is still outside the main playback mission path. Its dormant turn-in now requires chassis motion and measured yaw; stale inputs and timeouts fail, and phase boundaries publish zero.

### Deployment, dashboard, and tests

- `competition.launch.py` delegates to the same bringup as normal operation; boom-gate perception is included.
- Existing robot camera launch files are preserved. A clean checkout can use the tracked Astra Pro XML launch with UVC support. The fallback has not been run on the robot.
- Duplicate legacy packages at the repository root have `COLCON_IGNORE` markers. The installer chooses explicit source paths and preserves an existing nested camera checkout.
- Autostart launches exactly one bringup and uses systemd process-group shutdown. It no longer independently launches duplicate camera/motor processes.
- Simulation relays safe normalized steering into the Gazebo plugin's yaw-rate convention and bridges `/clock`. BPU signage is explicitly optional in simulation; full mission tests need recorded perception topics and parking behavior, not invented detections.
- Dashboard parameter changes use the node's declared ROS type. New controls are exposed; recording paths are validated. Display odometry reset is labeled as a display operation.
- Health watches source sensors and signage validity. Regression validation requires real samples, retains bad loop samples, and uses timeout-counter deltas within its window.
- `tools/bpu_model/model_manifest.json` pins the current artifact, class order, and six-output decoder contract. Legacy YOLOv5 compilation requires an explicit legacy argument; it is not a YOLO11 retraining recipe.

## Command contract

`angular.z` on internal robot command topics means normalized steering, **positive right**, in [-1, 1]. It is not a ROS yaw rate. The simulator translates this contract at its actuator boundary.

`linear.x` is a requested speed mapped to motor duty with `motor_duty_per_mps`. Physical speed remains dependent on calibration, battery, and load; the software cap alone is not a measured-speed guarantee.

Recorded samples are converted into requests, then subjected to safety speed/acceleration limits. **Existing timed recordings must be revalidated**, because ramps or limits can change their traveled distance. Completion acknowledges replay, not successful geometric parking measured by a bay sensor.

## Configuration that still needs the robot

1. Measure actual footprint and LiDAR position/orientation. The default sweep uses half-width 0.12 m, front/rear 0.22 m, and a 0.15 m horizon, including conservative margins. Verify scan coverage and stopping distance. A planar LiDAR cannot guarantee detection of an object above or below its scan plane.
2. Calibrate motor duty versus measured speed, ticks per metre, servo center/ranges, steering maximum, and the right-turn boost. Servo geometry is published to the safety sweep, but this is still commanded geometry rather than a measured steering sensor.
3. Calibrate camera perspective and ground-plane spans before enabling metric Pure Pursuit. Compare sliding and scanline behavior on the actual white-bordered surface. Tune lookahead, gains, and speed from recorded runs.
4. Record and validate separate maneuvers, then set `servo_controller.parallel_recording` and `servo_controller.perpendicular_recording` to their saved names. They default to empty deliberately; no arbitrary recording is selected for a mission.
5. Verify roundabout sign visibility, branch selection, gate diversion, tunnel corner geometry, traffic-light release, and terminal parking on the replica. Adjust detection regions using observed data.
6. Capture and label the traffic-light dataset. Model hash/shape checks do not establish detection accuracy. Retraining/export for the current model requires the correct YOLO11 pipeline.

## Local validation

From the repository root:

```text
python -B -m unittest discover -s tests -p "test_*.py" -v
python tools/preflight.py
python tools/bpu_model/verify_bpu.py
```

The behavioral tests import production classes with inert ROS/motor interfaces. A package-level test runs them in a subprocess so stubs cannot leak into other ROS tests. Python compilation, model hash, package-source selection, dashboard JavaScript, and shell syntax are also checked locally.

No ROS build, BPU inference, live DDS integration, real stopping-distance measurement, or track run has been performed for this change set. Those remain required before calling the robot competition-ready.

## Robot validation sequence

1. Save the existing robot configuration and recordings; identify the actual workspace with `tools/preflight.py`. Do not pull the upstream template workspace.
2. Build from the preflight's explicit source paths. Include `control_servo` as well as `risabot_automode`; install the changed bundled motor library if that is the library used by the robot.
3. With wheels raised, verify zero at startup, e-stop, source loss, driver loss, playback interruption, correct steering directions, and clean return to manual after faults.
4. Verify measured clearance/stopping at low speed, then each mission segment individually.
5. Record timestamped full-lap data and compare contact count, false stops, completion, steering saturation, source age, and loop rates before increasing speed.

No commit, push, robot connection, deployment, or service installation was performed by the local implementation work.
