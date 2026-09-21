# Track autonomy validation runbook

This is the working checklist for promoting V4 from calibrated shadow mode to
supervised autonomous competition use. Check an item only when its evidence has
been saved. A green dashboard alone is not evidence that a physical gate has
passed.

## Current starting point

- [x] Existing mission, safety and actuator stack implemented.
- [x] V4 Stages 0-8 and guarded motion executor implemented.
- [x] Cameras, LiDAR, UWB and gamepad previously observed on the board.
- [x] Manual wheels-up directions checked.
- [x] Steering centre measured as `servo_center: 110` and committed.
- [ ] Primary camera profile calibrated.
- [ ] Secondary camera profile calibrated.
- [ ] V4 road thresholds validated on the competition surface.
- [ ] Vehicle geometry, turning radius and LiDAR extrinsics validated.
- [ ] Wheels-up V4 motion and fault-injection validation completed.
- [ ] Parallel and perpendicular parking validated.
- [ ] Complete supervised competition lap completed repeatedly.

## Responsibility split

### Aryan / physical team

- Charge batteries and control physical power.
- Freeze camera, LiDAR, servo and body mounts.
- Place checkerboard and measured ground markers.
- Measure body geometry, LiDAR pose and turning circles.
- Move the car by hand for recordings and supervise all powered trials.
- Hold the controller and perform manual takeover when requested.
- Place real signs, lamps, boom gates, tunnel walls and obstacles.
- Decide whether a physical trial passed after inspecting the car and course.

### Codex / software work

- Inspect the live ROS graph, rates, resolutions and duplicate publishers.
- Run or provide capture and recording commands.
- Calculate camera calibration and enter the measured profiles.
- Update calibration tests when placeholder profiles become measured profiles.
- Analyze bags and tune V4 road, corridor and trajectory parameters.
- Deploy changes, build packages and inspect V4 blockers.
- Monitor command topics during wheels-up and ground trials.
- Run controlled fault injections and verify fail-closed behavior.
- Commit and push each evidence-backed configuration change.

## Equipment to take to the track

- [ ] Fully charged robot battery.
- [ ] Fully charged gamepad/receiver.
- [ ] Robot power supply and charger.
- [ ] Laptop on the same network as the RDK X5.
- [ ] Printed checkerboard with 9 x 6 inner corners.
- [ ] Accurate checkerboard square measurement.
- [ ] Tape measure or steel ruler.
- [ ] Four high-contrast floor markers.
- [ ] Masking tape/chalk for turning-circle marks.
- [ ] Real traffic lights, signs and boom-gate examples where available.
- [ ] Physical wheel stand so all wheels can rotate without touching anything.
- [ ] A second person at the power switch during motion tests.

---

## Phase 1 - Charge, inspect and freeze the hardware

**Owner: Aryan**

- [ ] Charge the propulsion battery completely.
- [ ] Charge/pair the controller.
- [ ] Tighten the Astra mount; mark its position so movement is visible.
- [ ] Tighten both side-camera mounts.
- [ ] Tighten the LiDAR and verify it is level.
- [ ] Confirm steering linkage has no loose screw or changing trim.
- [ ] Confirm all wheels rotate freely and tyres are secure.
- [ ] Confirm the cooling fan runs and airflow is unobstructed.
- [ ] Record a photograph of every final sensor mount.

**Stop condition:** if a camera or LiDAR mount moves after calibration, repeat
the affected calibration.

---

## Phase 2 - Connect and establish a clean baseline

**Owner: Aryan connects hardware; Codex can inspect remotely.**

Use the board's current IP in place of `<BOARD_IP>`. The last known address was
`192.168.137.161`.

```bash
ssh sunrise@<BOARD_IP>
source /opt/tros/humble/setup.bash
source /home/sunrise/risabotcar_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/sunrise/risabotcar_ws/install/risabot_automode/share/risabot_automode/config/disable_shm.xml
```

- [ ] Check for an existing stack before starting another one:

  ```bash
  ros2 node list
  systemctl is-active risabot-cams.service
  cat /sys/class/thermal/thermal_zone0/temp
  ```

- [ ] Start the production stack with legacy command authority:

  ```bash
  ros2 launch risabot_automode bringup.launch.py autonomy_source:=legacy
  ```

- [ ] In another configured terminal, verify publishers and rates:

  ```bash
  ros2 topic info /camera/color/image_raw -v
  ros2 topic info /camera/second/image_raw -v
  ros2 topic info /scan -v
  ros2 topic info /odom -v
  ros2 topic info /joy -v

  ros2 topic hz /camera/color/image_raw
  ros2 topic hz /scan
  ros2 topic hz /odom
  ros2 topic hz /joy
  ```

- [ ] Verify live image dimensions:

  ```bash
  ros2 topic echo --once /camera/color/image_raw --field width
  ros2 topic echo --once /camera/color/image_raw --field height
  ros2 topic echo --once /camera/second/image_raw --field width
  ros2 topic echo --once /camera/second/image_raw --field height
  ```

- [ ] Verify manual driving with wheels raised.
- [ ] Unlock the controller with one button press, then centre both sticks.
- [ ] Verify Start/Y switches AUTO and MANUAL.
- [ ] Finish this phase in MANUAL.

**Pass evidence**

- One publisher per intended camera topic.
- Primary camera, scan, odometry and joystick update continuously.
- No duplicate application nodes.
- Manual takeover stops wheel motion.
- Temperature remains stable rather than continuously climbing.

---

## Phase 3 - Record the real competition surface

**Owner: Aryan moves the car by hand; Codex starts and checks recording.**

Do this in MANUAL. Prefer pushing the powered-off drivetrain by hand if that is
mechanically safe; otherwise use the lowest manual speed with a spotter.

```bash
mkdir -p ~/track_validation

ros2 bag record \
  -o ~/track_validation/track_raw_$(date +%Y%m%d_%H%M%S) \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/second/image_raw \
  /scan \
  /odom \
  /imu/data \
  /dashboard_state
```

Record all of the following:

- [ ] Straight lane in normal light.
- [ ] Left curve.
- [ ] Right curve.
- [ ] Tightest competition curve.
- [ ] Strong shadow across the road.
- [ ] Bright/glare condition.
- [ ] Junction or opening in the white boundary.
- [ ] Roundabout entry and exit.
- [ ] Ramp/hill approach and crest.
- [ ] Tunnel entry, inside and exit.
- [ ] Obstruction area.
- [ ] Boom gate open and closed.
- [ ] Red, yellow and green traffic lights.
- [ ] Every sign class.
- [ ] Parallel and perpendicular parking markings.

Target 5-10 minutes of useful footage. Stop with `Ctrl+C` and retain the bag
directory unchanged.

**Codex output:** topic-rate report, dropped-frame findings, representative
frames and an initial legacy/V4 tuning report.

---

## Phase 4 - Primary camera intrinsic calibration

**Owner: Aryan positions checkerboard; Codex captures and calculates.**

- [ ] Measure checkerboard square size precisely in metres.
- [ ] Keep the camera at its final resolution and mount position.
- [ ] Capture at least 20 sharp, distinct views.
- [ ] Include near, middle and far distances.
- [ ] Include left/right/top/bottom image regions.
- [ ] Include modest checkerboard tilt in both axes.
- [ ] Reject blurred images and repeated near-identical views.

Calibration command after images exist:

```bash
ros2 run risabot_v4_experimental calibrate_intrinsics \
  '/home/sunrise/calibration/primary/*.png' \
  --columns 9 \
  --rows 6 \
  --square-m <MEASURED_SQUARE_SIZE_M> \
  --camera primary
```

**Codex tasks**

- [ ] Review RMS error and rejected images.
- [ ] Inspect straight-line undistortion visually.
- [ ] Enter `resolution`, `camera_matrix` and `distortion_coefficients` in
  `src/risabot_v4_experimental/config/camera_profiles.yaml`.
- [ ] Leave `calibrated: false` until Phase 5 ground correspondences pass.

---

## Phase 5 - Ground-plane / BEV calibration

**Owner: Aryan measures; Codex enters and verifies.**

The origin is the midpoint of the rear axle. Forward is positive; left is
positive.

- [ ] Place four visible floor markers forming a large non-crossed quadrilateral.
- [ ] Measure each marker relative to the rear-axle midpoint:

  ```text
  Marker 1: forward ______ m, left ______ m
  Marker 2: forward ______ m, left ______ m
  Marker 3: forward ______ m, left ______ m
  Marker 4: forward ______ m, left ______ m
  ```

- [ ] Capture one raw primary-camera frame containing all four markers.
- [ ] Record the raw pixel centre `[u, v]` for each marker in the same perimeter order.

**Codex tasks**

- [ ] Enter `source_points_px` and `ground_points_m`.
- [ ] Set the measured primary profile to `calibrated: true`.
- [ ] Update the repository test that currently asserts placeholder profiles.
- [ ] Build and launch Stage 1:

  ```bash
  cd /home/sunrise/risabotcar_ws
  colcon build --symlink-install --packages-select risabot_v4_experimental
  source install/setup.bash
  ros2 launch risabot_v4_experimental stage1_bev.launch.py enabled:=true
  ```

- [ ] Inspect `/v4_experimental/bev/status` and BEV/coverage images.

**Pass criteria**

- Straight floor lines remain straight.
- All four markers land within approximately 2-3 cm of measured positions.
- Useful road area has real coverage; unobserved black regions are excluded.
- A wrong-resolution image is rejected.

---

## Phase 6 - Vehicle geometry, turning radius and LiDAR calibration

**Owner: Aryan measures; Codex updates configuration and tests.**

- [ ] Measure complete vehicle length.
- [ ] Measure maximum vehicle width.
- [ ] Measure rear-to-front axle wheelbase.
- [ ] Measure rear-axle centre to rear body edge.
- [ ] Select a conservative footprint padding, initially 5-10 mm.

Record:

```text
vehicle_length_m: ______
vehicle_width_m: ______
wheelbase_m: ______
rear_overhang_m: ______
footprint_padding_m: ______
```

### Turning radius

- [ ] At the lowest manual speed, drive a full-lock left circle three times.
- [ ] Repeat full-lock right three times.
- [ ] Measure the rear-axle-midpoint circle diameter.
- [ ] Divide diameter by two.
- [ ] Use the larger average radius as `minimum_turn_radius_m`.

### LiDAR pose

- [ ] Measure LiDAR forward offset from rear-axle midpoint.
- [ ] Measure LiDAR left offset from centreline.
- [ ] Determine yaw from live scan geometry, not only physical appearance.
- [ ] Place a flat target 0.5 m directly in front.
- [ ] Confirm its transformed location is positive forward and near zero lateral.

Only after evidence passes, Codex sets for `v4_trajectory_shadow`:

```yaml
vehicle_geometry_validated: true
minimum_turn_radius_validated: true
lidar_extrinsics_validated: true
```

Do not open the duplicate parking/recovery gates until their rear/side coverage
has also been calibrated.

---

## Phase 7 - Tune V4 road and trajectory in shadow mode

**Owner: Codex tunes; Aryan supplies live track placement and confirms geometry.**

Start production sensors with legacy authority, then the lane-only shadow:

```bash
ros2 launch risabot_automode bringup.launch.py autonomy_source:=legacy
```

```bash
ros2 launch risabot_v4_experimental lane_shadow.launch.py enabled:=true
```

- [ ] Inspect `/v4_experimental/bev/status`.
- [ ] Inspect `/v4_experimental/road/status`.
- [ ] Inspect `/v4_experimental/trajectory/status`.
- [ ] Tune dashboard group **V4 Lane - Road Mask**.
- [ ] Tune **V4 Lane - Trajectory**.
- [ ] Replay recorded straight, curve, shadow and junction cases.
- [ ] Confirm connected road never leaks through open background.
- [ ] Confirm corridor follows the intended branch at junctions.
- [ ] Confirm candidate footprint remains inside observed road.
- [ ] Confirm LiDAR obstacle points reject intersecting candidates.
- [ ] Save accepted settings as defaults.

After repeated evidence passes:

```yaml
v4_road_mask_shadow:
  ros__parameters:
    thresholds_validated: true
```

The dashboard cannot and should not set this validation gate.

---

## Phase 8 - Validate competition perception and specialized behaviors

**Owner: Aryan places real props; Codex monitors classifications and states.**

- [ ] Unified14 model hash on board matches manifest.
- [ ] Red light stops and remains stopped.
- [ ] Yellow light follows the required competition behavior.
- [ ] Green light releases the wait.
- [ ] Unknown/unresolved lamp never causes unsafe release.
- [ ] Boom closed is blocked.
- [ ] Boom open allows progression.
- [ ] Every sign class is detected at realistic distance and angle.
- [ ] Tunnel entry/exit and wall following pass at crawl speed.
- [ ] Obstruction detection and avoidance pass with a soft obstacle.
- [ ] Roundabout state sequence is correct.
- [ ] Hill detection and speed cap are correct.

Save a bag and result note for every failed and successful case.

---

## Phase 9 - Parking readiness

The current legacy recording paths are empty. A complete mission cannot be
accepted until parking is solved.

- [ ] Choose the competition plan: measured legacy recordings or calibrated V4 parking.
- [ ] Calibrate the secondary camera if V4 parking/recovery will be used.
- [ ] Measure both parking-bay dimensions.
- [ ] Validate rear/side camera coverage.
- [ ] Record/plan parallel parking.
- [ ] Record/plan perpendicular parking.
- [ ] Test each maneuver wheels-up.
- [ ] Test each maneuver at crawl speed with soft boundaries.
- [ ] Require repeatable full containment and a clean exit.

No historical recording should be reused without validation under the current
servo centre, speed ramps and command contract.

---

## Phase 10 - Wheels-up V4 command-chain validation

**Owner: Codex operates software; Aryan and a spotter control physical safety.**

Preconditions:

- [ ] Wheels securely raised.
- [ ] Controller connected and in Aryan's hand.
- [ ] Second person at the power switch.
- [ ] V4 lane speed set to 0.04-0.06 m/s while authority is still closed.
- [ ] Primary calibration and Stage 4 candidates pass.

After code review/test evidence, Codex opens the non-physical Stage 7 and Stage
8 gates needed for this named trial. `physical_trials_validated` stays false.
`operator_motion_authorized` is true only for the supervised session.

Launch in three terminals:

```bash
ros2 launch risabot_automode bringup.launch.py autonomy_source:=v4
```

```bash
ros2 launch risabot_v4_experimental lane_shadow.launch.py enabled:=true
```

```bash
ros2 launch risabot_v4_control stage8_control.launch.py enabled:=true
```

Monitor:

```bash
ros2 topic echo /v4_control/status
ros2 topic echo /cmd_safety_status
ros2 topic echo /cmd_vel_v4_raw
ros2 topic echo /cmd_vel_auto
```

- [ ] Output remains zero in MANUAL.
- [ ] Correct forward wheel direction in AUTO.
- [ ] Correct left/right steering sign.
- [ ] Centred corridor produces near-centred steering.
- [ ] Manual takeover immediately stops autonomous wheel motion.
- [ ] No V4 node publishes directly to the hardware command interface.

---

## Phase 11 - Fault injection and stop validation

Keep wheels raised.

- [ ] Assert e-stop:

  ```bash
  ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
  ```

- [ ] Confirm `/cmd_vel_auto` becomes zero and wheels stop.
- [ ] Release only after inspection:

  ```bash
  ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
  ```

- [ ] Stop the V4 lane launch; confirm timeout stops output without legacy fallback.
- [ ] Remove primary camera input; confirm motion permission becomes false.
- [ ] Remove LiDAR input; confirm motion permission becomes false.
- [ ] Disconnect controller; confirm stop and MANUAL within the configured timeout.
- [ ] Reconnect controller; confirm button-unlock and neutral-stick gate.
- [ ] Inject stale/invalid proposal data; confirm zero output.

After every item passes, record the evidence and set
`physical_trials_validated: true`. Set `operator_motion_authorized: false` when
the session ends.

---

## Phase 12 - Ground crawl and speed ramp

**Owner: Aryan supervises; Codex records and monitors.**

- [ ] Begin on a clear straight at 0.04 m/s.
- [ ] AUTO for only 1-2 m, then return to MANUAL.
- [ ] Repeat until steering direction and centring are consistent.
- [ ] Test a gentle left curve.
- [ ] Test a gentle right curve.
- [ ] Test the tightest curve.
- [ ] Test a shadow and junction.
- [ ] Require five consecutive clean runs at each stage.
- [ ] Increase to 0.06 m/s only after reviewing evidence.
- [ ] Increase to 0.08 m/s only after another clean series.

Record commands and perception:

```bash
ros2 bag record \
  -o ~/track_validation/v4_ground_$(date +%Y%m%d_%H%M%S) \
  /camera/color/image_raw \
  /scan \
  /odom \
  /dashboard_state \
  /v4_experimental/road/status \
  /v4_experimental/trajectory/status \
  /v4_experimental/arbitration/status \
  /v4_control/status \
  /cmd_safety_status \
  /cmd_vel_v4_raw \
  /cmd_vel_auto
```

Reject a run for boundary contact, oscillation, wrong steering, prolonged
corridor loss, unexplained stop, failed takeover or thermal throttling.

---

## Phase 13 - Challenge segments and complete laps

- [ ] Run each challenge separately at crawl speed.
- [ ] Run Lap 1 without optional speed increases.
- [ ] Review bag, event log and stop reasons.
- [ ] Run Lap 2 including both parking behaviors.
- [ ] Review again.
- [ ] Run a complete two-lap mission with a safety operator.
- [ ] Repeat complete mission at least three times without intervention.
- [ ] Repeat after reboot to verify startup reproducibility.
- [ ] Confirm final temperature and loop rates remain stable.

Only after this phase should the competition launch be considered validated:

```bash
ros2 launch risabot_v4_control v4_competition.launch.py \
  autonomy_source:=v4 \
  v4_enabled:=true
```

Start every run in MANUAL. Inspect V4 and safety blockers before selecting AUTO.

---

## Phase 14 - Freeze and preserve the competition configuration

**Owner: Codex prepares; Aryan confirms the physical setup.**

- [ ] Commit measured camera profiles and matching tests.
- [ ] Commit road/trajectory parameters.
- [ ] Commit geometry and LiDAR extrinsics.
- [ ] Commit validated parking files or paths.
- [ ] Save model hashes and deployed commit ID.
- [ ] Export the best successful bags/replays.
- [ ] Create a board backup.
- [ ] Print/save the one-command startup and emergency shutdown procedure.
- [ ] Return `operator_motion_authorized` to false in the stored configuration.
- [ ] Keep a proven legacy-authority launch available as fallback.

## Gate rule

Open each measurement gate immediately after its own evidence passes. Keep the
motion gates closed until wheels-up tests. Keep operator authorization closed
except during a named supervised session. Never open all gates merely to make a
dashboard status turn green.
