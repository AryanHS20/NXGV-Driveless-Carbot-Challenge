# Risabot 1 battery reconnect: staged deployment and test

## Latest deployment: live perception controller (18:18 local)

**Physical trial failed:** the operator reported straight travel about 20 cm then
right drift. Recorder `20260923T101809Z` captured AUTO for 11.27 seconds, followed
by MANUAL and zero drive. Initial right steering later changed to a small left
correction before planning stopped. No further AUTO trial is ready. The operator
confirmed MANUAL forward-only driving also drifts right and the front wheels are
misaligned. Check common steering offset versus nonparallel wheels, then correct
the neutral/physical alignment before AUTO. See the live controller report for
frame-age and camera-width findings.

R1 now runs `lane_controller=live_lane_arc`. It checks the live pure-pursuit
command directly and searches other checked arcs only when that command fails.
The earlier polynomial/recovery steering chain below describes the preceding
deployment. R1 recorded playback is now explicitly disabled; the live ROS query
returned `allow_recorded_playback=false`, and the dashboard reported `IDLE`.

58 targeted tests pass. Camera streaming and new controller frames were verified
after restart. On the operator-confirmed centered straight, 14/15 dashboard
samples had a valid unblocked selection; one had a stale-mask blocker. All 15
were MANUAL with zero motor. Do not describe this as a successful physical drive.

The recorded obstacle-image check took about 12 ms on the board and found a
supported -0.03623 rad command. Alternative search can take about 210 ms.
The physical steering/geometry issue remains under test. Roundabout exits and
lane-change branch selection are still outside this lane-only launch.

Current supervised-test recorder:
`/home/sunrise/track_test_run/trials/20260923T101809Z`.
Deployment backup:
`/home/sunrise/risabot1_track_ws/live_arc_backup_20260923T101126Z`.
Detailed changes: `../risabot1_snapshots/20260923_live_arc/IMPLEMENTATION.md`.

## Prepared while the car is off

- Target: `sunrise@192.168.137.161`, dashboard `http://192.168.137.161:8080/`, ROS domain `1`.
- Active service is expected to run `track_test.launch.py` in MANUAL with `vehicle:=risabot1`, `motor_duty:=65`, `minimum_turn_duty:=40`, and the installed R1 camera profile. Confirm this again on connection.
- R1 track-test configuration enforces the swept road-support gate, disables the 0.6 s plan hold, sets road/mask freshness to 0.35 s, and sets minimum road support to 0.96. The reviewed file was built on Risabot 1 and its live 0.96 parameter was confirmed after the board restarted. R5 settings remain as before.
- The saved AUTO bend log had 65 selected reports marked valid; all 65 had blocked steps for the steering command actually sent. At 1.05 s the command had 55 blocked steps and zero minimum support. The saved 6.4 s image shows the dark road turning out of the camera view. A strict gate would stop there; it cannot create a visible route.
- ROS-free checks: 47 relevant tests passed on the laptop, including a Risabot 1 trajectory-to-arbitration stop test. The compact log checker is `tools/audit_risabot1_trajectory_log.py`.

## When the charged battery is connected

1. Keep the joystick in MANUAL and the car physically restrained for the first startup check. Confirm the dashboard reports MANUAL, E-stop and command output are understood, and camera/LiDAR/IMU data are fresh. Do not switch to AUTO during deployment.
2. Reconnect by SSH. Confirm hostname `risabot1`, service command, ROS domain, installed package paths, current source hashes, and active ROS parameters. The last live check showed the board `trajectory_core.py` differed from the laptop copy. Preserve that board file and **do not bulk-sync the repository**. Compare the board's `track_test_config.py` with the pre-deployment snapshot before copying anything.
3. Back up the board's `track_test_config.py` under a timestamped name and save the current service status. Copy only the reviewed local `src/risabot_v4_control/risabot_v4_control/track_test_config.py` to a temporary board path, compare it with the board file, then install it into `/home/sunrise/risabot1_track_ws/src/src/risabot_v4_control/risabot_v4_control/`. Build `risabot_v4_control` in `/home/sunrise/risabot1_track_ws` with `colcon build --packages-select risabot_v4_control --symlink-install`, then restart `risabot1-track-stack.service` while still in MANUAL. If the remote baseline changed, reconcile it first.
4. With `ROS_DOMAIN_ID=1` and the track workspace sourced, verify `/v4_trajectory_shadow` has `enforce_road_support_in_track_test=true`, `plan_hold_sec=0.0`, `road_timeout_sec=0.35`, `minimum_road_support=0.96`; verify `/v4_arbitration_shadow lane_only=true`. Check there is one final motor/servo owner, no autonomous command in MANUAL, and dashboard/ROS status show no stale sensor.
5. Start `python3 tools/record_autonomy_test.py --continuous` on the board and wait for `recording_ready`. While the operator drives **MANUAL** at the intended bend, capture continuous camera, mask, trajectory, sign, LiDAR, IMU, and final command. Keep the car centered and note the first position at which the road disappears from the camera. Stop the recorder cleanly and copy the bag and `events.jsonl` for analysis.
6. Run `python3 tools/audit_risabot1_trajectory_log.py <events.jsonl> --require-safe` on the new recording. Inspect the first unsupported or missing path together with the image. If no supported trajectory is available across the bend, correct camera coverage or perception before an AUTO bend test. UWB and the 14-class sign detector do not fill a missing road corridor.
7. If stationary and MANUAL evidence shows supported commands through the visible straight/bend, perform one short supervised AUTO segment, with the operator ready to return to MANUAL immediately. Stop on the first line contact, wrong-way steering, stale sensor, or unexplained stop. Repeat the same segment only after inspecting its recording. Lane change and roundabout are separate later gates; current `lane_only=true` does not choose their branches.

## Rollback

Restore the timestamped board `track_test_config.py`, rebuild `risabot_v4_control`, and restart the service in MANUAL. Confirm the old parameter values and zero command. Keep the original board `trajectory_core.py` and camera profile throughout this first deployment.

## Scope and decision

This is a safety and evidence deployment, not a complete competition mission. The live trial showed rightward physical drift even while the short predicted trajectory stayed on the observed road; the planner then stopped when the path disappeared. The next gate is measured steering and camera alignment on a clear straight. Sign speed cues, route-aware turns, and roundabout exits must wait until actual yaw and lane position agree with the predicted motion over repeated short segments.

## Live findings from 23 September

- The deployed R1 configuration has SHA256 `f721a2741762c0e046761b2f8639833a89c501e6caa2af6f76bf5cabeff87982` on the board. The launch parameter file and live ROS query both showed minimum road support `0.96`. The service was active, and the dashboard showed MANUAL with zero drive command after the board restarted.
- After the restart, the camera driver had no image because USB exposed only the Orbbec `2bc5:0501` color interface. Reconnecting its USB plug brought back the `2bc5:0403` interface; the driver then opened the camera and BEV, road, and trajectory frames resumed. Check for both USB interfaces if this recurs.
- The stationary MANUAL obstacle-sign recording is at `../risabot1_snapshots/20260923_obstacle_sign_stop/`. Its audit found 763 trajectory reports, 272 valid selections, zero unsafe-valid selections, and 491 reports with no selection. After reaching the sign at about 59 s, there were zero valid selections through the end at about 182 s. Waiting at the sign did not make a turn path appear.
- On the obstacle approach, the mask and corridor contained safe candidate trajectories, yet `boundary_recovery_active` forced approximately `-0.30 rad` steering for an estimated lateral error around `-0.03 m`. The actual commanded-steering rollout crossed the road mask on 49 of 55 steps. A replay without that recovery still produced approximately `+0.176 rad`, also unsupported. A small command near straight was supported by the static mask, but that alone does not establish a safe physical turn. This is a controller/geometry mismatch; do not run AUTO through the obstacle approach or roundabout yet.
- For a bounded AUTO test, first position the car on a clear, centered straight with at least 1 m of lane visible. Confirm repeated fresh, valid selected paths and zero road-blocked sent-command steps while still in MANUAL. Stop the test before the obstacle approach.
- A later read-only dashboard recording is at `../risabot1_snapshots/20260923_live_reconnect/short_trial.jsonl`. The operator clarified that the observed straight movement was in MANUAL, before AUTO. During the captured AUTO interval the final drive command was always zero and the state was `LANE_RECOVERY` / `WAITING FOR V4 LANE`. The car did not drive autonomously. The car had moved after the preceding stationary path check, so that check did not validate its new starting position.
- A temporary live test of minimum road support `0.95` gave zero accepted paths in 60 stationary samples and was not persisted. The battery connector was then moved and the board rebooted, restoring the saved `0.96` setting. The camera and MANUAL zero command returned. Do not infer a successful AUTO trial from the MANUAL movement.
- A controlled retry after lifting the car to a clear straight passed 50/50 stationary MANUAL path checks: minimum sent-command support `1.0`, zero blocked steps, minimum estimated boundary clearance `0.04025 m`. The retry is recorded in `../risabot1_snapshots/20260923_live_reconnect/short_trial_retry.jsonl`.
- AUTO briefly entered `LANE_FOLLOW` and published nonzero drive with normalized steering about `-0.055` to `-0.065` (the trajectory selected a small left-positive steering angle, about `+0.048` to `+0.057 rad`). The operator saw the car move a few centimetres, drift right toward the white stripe, then stop. It entered `LANE_RECOVERY` and zero drive when the trajectory was rejected. The operator returned to MANUAL. Do not attempt another AUTO segment, turn, or roundabout from this result.
- A wheel-off-floor MANUAL check found neutral wheels visually straight and left joystick input pointing the wheels left. Code maps the AUTO negative steering command to that same servo direction, so the physical right drift is not yet explained by a simple joystick polarity reversal. The next calibration needs synchronized actual wheel angle, IMU yaw, mask and final command during a bounded straight drive, with the car physically restrained and no turn attempt until the measured response agrees with the model.
