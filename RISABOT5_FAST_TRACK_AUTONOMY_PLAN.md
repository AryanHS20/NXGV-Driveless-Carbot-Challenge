# Risabot 5 fast-track autonomy plan

Date: 2026-09-22

## Decision

Use the current `NXGV-Driveless-Carbot-Challenge` lane stack for the next physical test. It is already deployed to Risabot 5, its front-camera calibration is selected, manual steering works, and stationary left/right displacement checks requested corrections toward the lane centre.

Do not switch the first physical test to `NxGV-Carbot-Autonomous-V2`. V2 has the better full-course architecture, but it is not race runnable yet: `race_supervisor.py` refuses every START request and `calibration_wizard.py` refuses its actions because phase 8 is still a stub. Its default calibration contract also requires three cameras and UWB.

The deadline strategy is therefore:

1. Prove reliable supervised lane following with the current deployed stack.
2. Freeze the successful R5 hardware and front-camera settings.
3. Use V2 for the full competition mission after adding a small working race/start layer and an explicit reduced-sensor profile.
4. Add side cameras and UWB only after forward navigation is repeatable, unless a specific scored challenge requires them.

Do not run both autonomy stacks together. Exactly one node must own the final autonomous motor/steering command.

## What is required for the first autonomous lane test

Required:

- Risabot 5 battery charged and secured.
- Current front Astra RGB camera, unchanged mount and `native320` R5 calibration profile.
- Manual steering and MANUAL takeover.
- Current V4 road mask, local trajectory, arbitration and motion-executor chain.
- Fresh camera frames and one publisher for each critical command topic.
- The requested 65% motor-duty cap.
- A short, bounded course segment with a person holding the controller beside the car.

Not required:

- UWB.
- Side cameras.
- Global map or mission route.
- Parking and recovery.
- The new dashboard.
- Traffic-light, boom-gate or sign detection.

Keep the essential stop paths: MANUAL takeover, e-stop/stop command, stale camera/command watchdog and one command owner. These do not reduce steering authority; they prevent an old or conflicting command from continuing.

## Battery-ready sequence

### 1. Reconnect and verify MANUAL

- Confirm the board IP and SSH.
- Confirm the deployed file hashes still match the deployment manifest.
- Start one lane-test stack in MANUAL.
- Confirm the selected vehicle is Risabot 5, servo centre is 80, profile is R5 `native320`, and the motor command remains zero.
- Confirm fresh camera, road-mask, trajectory and arbitration messages.
- Confirm one active autonomous command owner and no concurrent V2/base auto-driver process.

### 2. Verify stopping before floor motion

With drive wheels supported, briefly select AUTO and immediately return to MANUAL. Confirm the motor stops and AUTO does not re-enable itself. Also stop the camera or executor once and confirm the watchdog produces zero drive. Restore the stack in MANUAL.

### 3. Run four short floor trials at the requested 65% duty cap

Record every trial. Begin each one in MANUAL and deliberately select AUTO only after the car is placed.

1. **Centred straight:** 0.5-1.0 second or about 1 metre, whichever comes first. Return to MANUAL.
2. **Slightly left of centre:** car parallel to the lane. It must steer right and reduce the error.
3. **Slightly right of centre:** car parallel to the lane. It must steer left and reduce the error.
4. **One gentle bend each direction:** verify useful steering authority without sustained left-right oscillation.

Stop a run immediately if the correction sign is wrong, steering saturates without changing the path, the selected corridor jumps, or the car leaves the drivable surface.

### 4. Tune from evidence

Change one parameter group at a time:

- Persistent left drift while the perceived corridor is centred: adjust steering trim/centre, not vision thresholds.
- Perceived corridor consistently shifted: correct camera mount/homography bias.
- Correct correction direction but too little steering: increase lateral/heading correction gain or steering mapping within measured range.
- Repeated overshoot: reduce correction gain or add damping/lookahead.
- Image-edge or shadow attraction: tune road-mask classification and candidate support; do not hide it with steering trim.

After every change, repeat the centred, left-offset and right-offset trials. Freeze a configuration only after three successful short repeats.

## UWB decision

UWB is not needed for lane-centering, short-horizon steering, local obstacle avoidance or the first track test. V2's local pose and safety monitor do not depend on UWB; its safety code explicitly excludes UWB.

UWB helps the full mission identify the car's coarse position on the course, recover accumulated odometry error and validate the start pose. It is useful for repeated full laps and route/checkpoint identity, but it should be deferred until the car can already drive the visible lane reliably.

For the deadline build, initialize at a known marked start pose and use encoder/IMU local odometry plus visual map correction. Add UWB later if route identity drifts or the competition requires dependable global relocalization.

## Side-camera decision

Side cameras are not needed for forward lane following. V2 road perception can produce a grid from the fresh camera subset, although its current status and preflight expect all three cameras.

Side/rear views are valuable for:

- observing road beside and behind the car during reverse recovery;
- seeing parking-bay tape and swept space that the front camera cannot see;
- keeping local road memory valid through parking and tight manoeuvres.

Defer them for the first forward-navigation milestone. Before enabling computed reverse parking or recovery, either calibrate the side cameras or use a separately validated parking method allowed by the competition rules. Do not label a recorded manoeuvre as perception-planned parking.

## Architecture audit

### Current repository: good for the immediate lane milestone

The active lane path is coherent:

`front camera -> calibrated BEV/road mask -> corridor -> trajectory candidates -> arbitration -> motion executor -> command safety -> servo controller`

Strengths:

- already runs on Risabot 5;
- front-camera calibration and steering centre are vehicle specific;
- one lane command path with MANUAL ownership and watchdog stops;
- candidate planner now applies a stronger centre correction;
- offline regression and stationary direction checks pass.

Limits for full competition autonomy:

- the full launch does not consistently select the R5 profile;
- mission route selection is not connected to the V4 trajectory branch choice;
- normal mission parking still selects recorded playback rather than the V4 planner;
- parking/recovery execution lacks enough closed-loop lateral and heading correction;
- local/coarse pose, map activation and recovery-result contracts remain incomplete;
- several dashboard actions do not yet have real runtime adapters.

Finishing all of those in this repository is slower than the first lane test and duplicates work already present in V2.

### V2 repository: better full-course structure, incomplete race shell

V2 has the better final architecture:

- one `command_owner` for autonomous output;
- separate smooth local pose and coarse global pose;
- map, global route, mission state, corridor, local planner, parking, recovery and path tracker;
- front perception, local road memory, LiDAR tunnel/obstacle handling and detector inputs;
- session-based configuration and calibration overlays.

Current blockers:

- `race_supervisor.py` is a phase-8 stub and refuses START unconditionally;
- `calibration_wizard.py` is a phase-8 stub and refuses actions;
- the default required steps demand all three cameras and UWB;
- camera roles/extrinsics, UWB alignment, speed feed-forward and other R5 values are not calibrated in the checked-in defaults;
- camera launch is all-or-none, rather than separate front and side switches;
- the full stack has not been physically validated on Risabot 5.

## Minimal V2 work after lane following passes

Implement only the pieces needed to make V2 executable on R5:

1. Add named sensor profiles: `front_nav` and `full_course`.
2. Split `start_front_camera` from `start_side_cameras`; allow `start_uwb_agent:=false`.
3. Make calibration requirements profile aware. `front_nav` must require front intrinsics/extrinsics, LiDAR alignment, odometry/IMU, steering and speed; it must report side/UWB as intentionally disabled rather than falsely passed.
4. Implement the race supervisor's real preflight and START service, defaulting to MANUAL/disarmed until an explicit start.
5. Port the measured R5 servo centre, geometry, front-camera calibration and low-level topic/sign conventions into a calibration session.
6. Calibrate encoder scale, IMU yaw, turning radius and duty-to-speed. The checked-in V2 feed-forward value is a placeholder.
7. Replay/simulate a complete route, then run straight, bends, roundabout exit, tunnel, obstacle and mission transitions one segment at a time.
8. Add side-camera calibration for computed parking/recovery. Add UWB only if full-course route identity needs it.

This keeps the strong V2 architecture without blocking initial autonomy on sensors that the first milestone does not use.

## Dashboard parallel work

The dashboard task can continue without touching the physical control path. It should consume recorded/sample status, expose stale/unavailable values honestly, and avoid deploying launch, command-owner, calibration or vehicle-profile files. After the control stack is frozen, connect dashboard buttons through acknowledged runtime services. The dashboard is not a prerequisite for the first AUTO floor trial.

