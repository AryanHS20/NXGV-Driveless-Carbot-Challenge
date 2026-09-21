# Track autonomy validation runbook

This is the working checklist for promoting V4 from calibrated shadow mode to
supervised autonomous competition use. Check an item only when its evidence has
been saved. A green dashboard alone is not evidence that a physical gate has
passed.

## Live session status — 2026-09-21 21:40 MYT

This section is the authoritative handover point. Update it immediately after
every completed test, including the evidence and any temporary override.

**Current safety state:** `MANUAL`; wheels raised; autonomous output held at
zero. The rebuilt V4 lane pipeline and temporary commissioning control launch
are running. Stage 4 now has a valid live centre candidate, but Stage 7 requests
a hard stop until the operator deliberately selects AUTO.

**Temporary commissioning changes on the board:**

- Stage 4, Stage 7 and Stage 8 validation gates were opened only through
  `/tmp/front_v4_*_commission.launch.py`; repository defaults remain closed.
- `cmd_safety_controller.require_signage` was temporarily set to `false` and
  the BPU signage node was stopped to reduce temperature during the lane-only
  wheels-up test. Restore both before any ground or mission test.
- Side cameras are off for this lane-only test.

### Proven in this session

- [x] Gamepad detected as `ShanWan Gamepad`, with `/dev/input/js0` and live
  `/joy` messages.
- [x] Manual forward, reverse, left, right and neutral stop physically verified
  with the wheels raised.
- [x] Joystick-loss watchdog stopped and relocked the controller after 1.1 s.
- [x] Unified14 BPU model hash matched
  `e1cc941f8bb320fb221c9422bef2a7daf86c1b7259fca466a49d667a2d394b6e`.
- [x] Unified14 loaded successfully and published `/signage_valid: true`.
- [x] Camera, calibrated BEV, connected road corridor and LiDAR were live.
- [x] Stage 8 isolated board test passed bounded-command, malformed-input,
  e-stop preemption and proposal-timeout behavior. E-stop reached zero in about
  20 ms in the isolated ROS domain.
- [x] Planner blind-strip defect identified: the Astra first observes road at
  about 0.45 m, while the old planner required image evidence beneath the car.
- [x] Bounded near-field bootstrap implemented. It refuses an excessive blind
  gap and a corridor narrower than the padded vehicle.
- [x] Local V4 tests pass: 73/73.
- [x] Updated `risabot_v4_experimental` package built successfully on the RDK
  X5. Board `colcon test` discovered zero tests, so local 73/73 is the current
  test evidence and board test discovery still needs correction.
- [x] Rebuilt Stage 4 loaded on the board and produced a valid live centre
  candidate: offset `0.0 m`, support `1.0`, steer `0.02132 rad`, no road or
  obstacle rejection, on 2026-09-21.
- [x] Diagnosed the unresponsive AUTO button: the first commissioning launches
  omitted `FASTRTPS_DEFAULT_PROFILES_FILE`, causing Fast DDS SHM errors and a
  dropped servo-to-mission mode handoff. Both launch trees were restarted cleanly
  with the repository UDP-only profile; the new logs contain zero SHM errors.
- [x] Diagnosed repeated AUTO-to-MANUAL fallback after the clean restart: the
  restart restored `require_signage: true` while the BPU node was intentionally
  off. For the wheels-up lane-only test it was reset to `false`; live
  `/motion_permitted` then reported `true`. Ground/mission testing must restore
  signage and `require_signage: true`.

### Do next — do not skip or reorder

- [x] Restart the rebuilt V4 lane pipeline and replace the pre-build Stage 4
  process with the rebuilt executable.
- [ ] Confirm exactly one publisher for each critical topic and remove any
  remaining duplicate ROS nodes.
- [x] Confirm Stage 4 produces a fresh `selected_diagnostic_only.valid: true`
  trajectory on the live track.
- [ ] Confirm Stage 7 publishes a fresh `follow_curvature` request.
- [ ] Keep wheels raised, press `B` once to unlock, then press `Y` or `Start`
  once to select AUTO.
- [ ] Verify correct autonomous forward-wheel direction and steering direction.
- [ ] Press `Y` or `Start` once and verify immediate MANUAL takeover and stop.
- [ ] Perform the physical e-stop and executor-timeout tests with wheels raised.
- [ ] Return to MANUAL and close temporary motion authority.
- [ ] Restore `require_signage: true`, restart unified14, and verify temperature.
- [ ] Commit and push the near-field bootstrap only after the live trajectory
  and wheels-up results pass.

### Still required before ground autonomy

- [ ] Measure vehicle length, width, wheelbase, rear overhang, minimum turning
  radius, and LiDAR `x/y/yaw`. V4 currently says wheelbase `0.216 m`; the legacy
  servo configuration says `0.14 m`, so this must be resolved physically.
- [ ] Validate road thresholds across straight, left/right curves, shadows,
  glare and junction openings.
- [ ] Validate unified14 against real red/yellow/green lights, every required
  sign, and boom-open/boom-closed examples.
- [ ] Keep the complete lane stack thermally stable. Observed temperature was
  about 76 C without BPU signage and about 82 C with it.
- [ ] Complete the crawl, challenge-segment, parking and full-lap phases below.

### Update rule

For each checkbox, record the date, measured result, relevant topic/log/bag,
and final PASS or FAIL. A failed attempt remains unchecked and its reason is
added beneath the item. Temporary overrides never count as stored competition
configuration.

## Current starting point

- [x] Existing mission, safety and actuator stack implemented.
- [x] V4 Stages 0-8 and guarded motion executor implemented.
- [x] Cameras, LiDAR, UWB and gamepad previously observed on the board.
- [x] Manual wheels-up directions checked.
- [x] Steering centre measured as `servo_center: 110` and committed.
- [x] Primary camera ground profile calibrated and held-out board corners validated.
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
- Place and measure ground markers; provide a checkerboard only if factory
  camera calibration fails validation.
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
- [ ] At least 10 GiB free storage on the board for the recording session.
- [ ] Optional fallback: printed checkerboard with 9 x 6 inner corners.
- [ ] Optional fallback: accurate checkerboard square measurement.
- [ ] Tape measure or steel ruler.
- [ ] At least six high-contrast floor markers.
- [ ] Masking tape/chalk for turning-circle marks.
- [ ] Real traffic lights, signs and boom-gate examples where available.
- [ ] Physical wheel stand so all wheels can rotate without touching anything.
- [ ] A second person at the power switch during motion tests.

---

## Phase 1 - Charge, inspect and freeze the hardware

**Owner: Aryan**

- [ ] Charge the propulsion battery completely.
- [x] Charge/pair the controller. Paired and live on `/joy` on 2026-09-21.
- [ ] Tighten the Astra mount; mark its position so movement is visible.
- [ ] Tighten both side-camera mounts.
- [ ] Tighten the LiDAR and verify it is level.
- [ ] Confirm steering linkage has no loose screw or changing trim.
- [x] Confirm all wheels rotate freely during the 2026-09-21 wheels-up manual test.
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

- [x] Verify manual driving with wheels raised.
- [x] Unlock the controller with one button press, then centre both sticks.
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

Start one evidence-complete session from the repository root. The recorder
refuses to start without the primary camera, LiDAR and odometry. It records all
available cameras, calibration messages, sensors, perception results, V4
statuses, safety decisions and command topics. It also saves parameters, the
Git revision, topic/node inventories and two-second system-health samples.

```bash
cd /home/sunrise/risabotcar_ws
bash tools/record_track_session.sh --name full_course_baseline --require-v4
```

Keep that terminal open. From a second configured terminal, mark the beginning
of each feature. Replace `<SESSION_DIR>` with the directory printed by the
recorder:

```bash
bash tools/mark_track_event.sh <SESSION_DIR> straight_normal
bash tools/mark_track_event.sh <SESSION_DIR> left_curve
bash tools/mark_track_event.sh <SESSION_DIR> shadow "strong cross-track shadow"
bash tools/mark_track_event.sh <SESSION_DIR> boom_closed
```

Use short event names containing the feature and state. A note is optional.
The marker timestamp allows the corresponding camera, LiDAR, perception and
control messages to be located precisely in the bag.

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

Target 5-10 minutes of useful footage. Mark every listed feature, stop the
recorder with `Ctrl+C`, and wait for `Session saved:` before disconnecting
power. Retain the complete timestamped session directory unchanged. Inspect
`metadata/verification.json`, `metadata/bag_info.txt` and `SHA256SUMS`; an
empty or missing required topic means the capture must be repeated.

**Codex output:** topic-rate report, dropped-frame findings, representative
frames and an initial legacy/V4 tuning report.

---

## Phase 4 - Primary camera intrinsics

The primary route uses the Astra's factory `CameraInfo`; a checkerboard is the
fallback. Factory intrinsics remove the need to own or print a target, but they
must still pass resolution and measured-ground validation.

### Phase 4A - Preferred factory-intrinsics route

**Owner: Aryan keeps the mount fixed; Codex validates and converts the data.**

- [x] Keep the camera at its final 320 x 240 resolution and mount position.
- [x] Start the Astra camera and verify the raw image is live.
- [x] Save the complete factory calibration message:

  ```bash
  mkdir -p ~/track_validation
  ros2 topic info /camera/color/camera_info -v
  ros2 topic echo --once /camera/color/camera_info \
    > ~/track_validation/astra_color_camera_info.yaml
  cat ~/track_validation/astra_color_camera_info.yaml
  ```

- [x] Save the live image dimensions for comparison (`320 x 240`):

  ```bash
  ros2 topic echo --once /camera/color/image_raw --field width
  ros2 topic echo --once /camera/color/image_raw --field height
  ```

**Codex validation**

- [x] `CameraInfo.width` and `height` exactly match the raw image.
- [x] `k` contains nine finite values with positive `fx` and `fy`.
- [x] Principal point `cx, cy` lies within or plausibly near the image.
- [x] `d` has a supported finite length: 4, 5, 8, 12 or 14 values.
- [x] Distortion model is compatible with OpenCV's calibration model, normally
  `plumb_bob`.
- [x] Factory values remain consistent across restarts at the same resolution.
- [x] Convert the row-major `k` list into the 3 x 3 `camera_matrix` and copy `d`
  into `distortion_coefficients`.
- [x] Leave `calibrated: false` until Phase 5 ground validation passes.

Captured evidence: `/home/sunrise/track_validation/20260921T090557Z_full_course_raw/metadata/camera_color_camera_info.yaml`.
The recorded raw stream and `CameraInfo` are both 320 x 240. The captured
matrix has `fx = fy = 285.17110237076486`, `cx = 159.5`, `cy = 119.5`, and
five finite zero distortion coefficients. These values are staged in
`camera_profiles.yaml`; they do not enable BEV while `calibrated` is false.
The same values were observed again after a clean camera-node restart on
2026-09-21 (`/tmp/astra_restart_camera_info.yaml` on the board).

Zero distortion coefficients are not automatically invalid. They are accepted
only if straight-line and held-out ground-marker tests pass.

### Phase 4B - Checkerboard fallback

Use this only if `CameraInfo` is absent, has zero/invalid focal lengths, has the
wrong resolution, changes unexpectedly, or fails the Phase 5 error test.

- [ ] Obtain a flat 9 x 6-inner-corner checkerboard. A correctly displayed,
  measured tablet/laptop pattern can be used if it is flat and free of glare.
- [ ] Measure checkerboard square size precisely in metres.
- [ ] Capture at least 20 sharp, distinct views.
- [ ] Include near, middle and far distances.
- [ ] Include left/right/top/bottom image regions.
- [ ] Include modest checkerboard tilt in both axes.
- [ ] Reject blurred images and repeated near-identical views.

```bash
ros2 run risabot_v4_experimental calibrate_intrinsics \
  '/home/sunrise/calibration/primary/*.png' \
  --columns 9 \
  --rows 6 \
  --square-m <MEASURED_SQUARE_SIZE_M> \
  --camera primary
```

**Codex fallback tasks**

- [ ] Review RMS error and rejected images.
- [ ] Inspect straight-line undistortion visually.
- [ ] Enter `resolution`, `camera_matrix` and `distortion_coefficients` in
  `src/risabot_v4_experimental/config/camera_profiles.yaml`.
- [ ] Leave `calibrated: false` until Phase 5 ground correspondences pass.

---

## Phase 5 - Ground-plane / BEV calibration

**Owner: Aryan measures; Codex enters and verifies.**

The primary Astra used the supplied 6 x 4 floor checkerboard instead of manual
tape points. Its measured 50.0 mm squares and board centre at `[0.765, 0.0]`
produced 24 detected corners. Four perimeter corners construct the homography;
the other 20 produced 11.8 mm RMS and 19.2 mm maximum ground error.

The origin is the midpoint of the rear axle. Forward is positive; left is
positive.

- [ ] Place at least six visible floor markers across the useful driving area.
- [ ] Choose four perimeter markers forming a large non-crossed quadrilateral
  for the homography. Reserve at least two as independent validation points.
- [ ] Measure every marker relative to the rear-axle midpoint:

  ```text
  Marker 1: forward ______ m, left ______ m
  Marker 2: forward ______ m, left ______ m
  Marker 3: forward ______ m, left ______ m
  Marker 4: forward ______ m, left ______ m
  Validation 1: forward ______ m, left ______ m
  Validation 2: forward ______ m, left ______ m
  ```

- [ ] Capture one raw primary-camera frame containing all markers.
- [ ] Record the raw pixel centre `[u, v]` of the four homography markers in
  the same perimeter order.
- [ ] Record validation-marker pixels separately; do not use them to construct
  the homography.

**Codex tasks**

- [x] Enter `source_points_px` and `ground_points_m`.
- [x] Set the measured primary profile to `calibrated: true`.
- [x] Update the repository test that currently asserts placeholder profiles.
- [x] Build and launch Stage 1:

  ```bash
  cd /home/sunrise/risabotcar_ws
  colcon build --symlink-install --packages-select risabot_v4_experimental
  source install/setup.bash
  ros2 launch risabot_v4_experimental stage1_bev.launch.py enabled:=true
  ```

- [x] Inspect `/v4_experimental/bev/status` and BEV/coverage images.

**Pass criteria**

- Straight floor lines remain straight.
- Four construction markers map consistently and every held-out validation
  marker lands within approximately 2-3 cm of its measured position.
- Straight physical edges remain straight after undistortion.
- If factory intrinsics fail either check, return to the checkerboard fallback.
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
wheelbase_m: 0.210  # measured rear axle centre to front axle centre
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

- [x] Inspect `/v4_experimental/bev/status`.
- [x] Inspect `/v4_experimental/road/status`.
- [x] Inspect `/v4_experimental/trajectory/status`.
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

- [x] Unified14 model hash on board matches manifest.
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

- [x] Wheels securely raised.
- [x] Controller connected and in Aryan's hand.
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

- [x] Output remains zero in MANUAL.
- [ ] Correct forward wheel direction in AUTO.
- [ ] Correct left/right steering sign.
- [ ] Centred corridor produces near-centred steering.
- [ ] Manual takeover immediately stops autonomous wheel motion.
- [x] No V4 node publishes directly to the hardware command interface.

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
- [x] Disconnect/pause controller input; confirmed stop, MANUAL and relock after
  1.1 s on 2026-09-21.
- [ ] Reconnect controller; confirm button-unlock and neutral-stick gate.
- [x] Inject stale/invalid proposal data; confirmed zero output in isolated
  board-domain Stage 8 test on 2026-09-21.

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
# 2026-09-21 V4 wheels-up autonomy result

- [x] Corrected mission lane readiness to follow the selected autonomy source.
  With `autonomy_source:=v4`, stale legacy `/lane_lost` no longer forces
  `LANE_RECOVERY`; a fresh, valid, unblocked V4 trajectory is required.
- [x] Vectorized Stage 4 footprint and obstacle evaluation. Board trajectory
  status increased from 4 samples/8 s to 34 samples/8 s under the same load.
- [x] Held the last valid matched road-mask/corridor pair only during the
  bounded split-message arrival interval; the 0.75 s stale stop remains.
- [x] Full raised-wheel V4 chain sustained AUTO after the fixes: over 12 s,
  `/cmd_vel_v4_raw` was nonzero for 598/598 samples, `/cmd_vel_auto` was
  nonzero for 550/550 samples, dashboard state was `LANE_FOLLOW` for 58/58
  samples, trajectory blockers were present for 0/49 reports, and there were
  zero command or state transitions.
- [ ] Restore and validate signage (`require_signage:=true`) before any full
  mission or competition run; it was deliberately disabled for this lane-only
  raised-wheel commissioning test.
- [ ] Measure/confirm wheelbase, minimum turning radius, and LiDAR pose before
  treating their temporary commissioning gates as physically validated.
- [ ] Perform supervised ground tests in order: 0.03-0.05 m/s straight crawl,
  gentle curves, shadows, lane-loss stop, obstacle stop, manual takeover, then
  challenge-by-challenge mission trials. Do not begin with a full-speed lap.
- [ ] Validate parking recordings and both parking manoeuvres on the final
  steering ramp before attempting a complete two-lap mission.
