# RISA-bot V4 experimental ROS package

This package is the isolated porting area for the algorithms demonstrated by
`Carbot_Simulator` V4. It is intentionally separate from `risabot_automode`.

## Safety boundary

- Nothing in the normal `risabot_automode` bringup launches this package.
- There is no motor-command publisher in this package.
- Every experimental output must remain under `/v4_experimental/*` until a
  reviewed integration step explicitly changes that contract.
- The existing controller remains the only vehicle command path.
- Reverse recovery is not enabled on the physical vehicle.

Stage 0 is a read-only shadow monitor. It checks whether the two camera feeds,
LiDAR and odometry are fresh and publishes JSON on `/v4_experimental/status`.
UWB can be required later, but is optional by default.

Stage 1 adds independently calibrated, metric bird's-eye transforms for the two
MIPI cameras. The checked-in profiles are explicit uncalibrated placeholders,
so the node refuses to publish BEV images until physical measurements are
entered. See `CALIBRATION.md`.

Stage 2 adds coverage-aware dark-road candidates,
four-connected growth from the vehicle seed, metric corridor samples, and a
2.5 cm odometry-fixed recent-road memory. Planning memory expires by age and
travel, and clears on odometry discontinuities. Its thresholds are explicitly
marked unvalidated and it cannot receive camera data until Stage 1 profiles are
calibrated.

Stage 3 mirrors `/odom` as the local pose and filters valid
`/v4_experimental/uwb/fix`
measurements into a separate coarse-global rigid transform. A UWB update
cannot rewrite the local pose. Innovation gating rejects gross outliers, and an
odometry discontinuity clears the coarse offset. UWB fusion remains gated until
the UWB and odometry frame alignment has been measured.

The separate Stage 3 UWB bridge converts unique `/uwb3/input_json` anchor
ranges into the `/v4_experimental/uwb/fix` contract. It is disabled by default
and blocked by anchor geometry, range offset, and antenna height validation
gates. See `UWB_INTEGRATION.md` for its measurement and board-validation
procedure.

Stage 4 generates nine short bicycle-model rollouts and checks the complete
vehicle footprint against the synchronized road mask plus fresh LiDAR points.
It publishes JSON candidate diagnostics only. Processing remains blocked until
camera calibration, road thresholds, body geometry, minimum turning radius,
and LiDAR extrinsics have each been measured and explicitly marked validated.

Stage 5 adds analytic Reeds-Shepp parking proposals with forward and reverse
segments. Every candidate is reconstructed with the bicycle model, bounded by
gear-change and reverse-distance limits, and checked against the secondary
camera parking-area mask and fresh LiDAR points using the complete footprint.
It requires a measured rear-axle goal described in `PARKING_GOAL.md`. No live
node supplies that goal yet, all physical gates remain false, and the package
cannot execute a proposed path.

Stage 6 adds bounded recovery proposals for cases where forward-only planning
has no valid candidate. A proposal must reverse 3--20 cm, change gear exactly
once, finish forward on the measured corridor, remain inside the complete
observed road footprint, avoid fresh LiDAR obstacles, and retain explicit rear
camera coverage. Any hard hold blocks planning. The request contract is in
`RECOVERY_REQUEST.md`; no live node supplies it, every validation gate remains
false, and this package still has no motion publisher.

The parking-goal source measures closed bright bay markings in the calibrated
secondary BEV image. The recovery-request source combines exhausted Stage 4
candidates, mission state, odometry speed, image time, and an attempt counter.
Both sources are disabled and their measurement/policy gates ship false.

Stage 7 performs diagnostic source arbitration between forward trajectory,
parking, recovery, and hard hold. It publishes JSON under
`/v4_experimental/arbitration/*`; it has no ROS motion-message dependency and
all integration, timeout, preemption, command-contract, and physical-trial
gates ship false.

Stage 8 is intentionally implemented in the separate `risabot_v4_control`
package so this package retains its no-motion contract. The executor converts
reviewed Stage 7 proposals into `/cmd_vel_v4_raw`; the production command
safety controller must explicitly select that source and still applies sensor,
e-stop, timeout, rate, speed, and swept-footprint checks. See the control
package README for the validation and launch procedure.

## Build only this package

```bash
cd ~/risabotcar_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select risabot_v4_experimental --symlink-install
source install/setup.bash
```

## Manual shadow run

The launch defaults to disabled. Explicitly enable it for read-only monitoring:

```bash
ros2 launch risabot_v4_experimental shadow.launch.py enabled:=true
ros2 topic echo /v4_experimental/status
```

Stopping the node has no effect on the existing driving stack.

The Stage 1 launch is also disabled by default:

```bash
ros2 launch risabot_v4_experimental stage1_bev.launch.py enabled:=true
ros2 topic echo /v4_experimental/bev/status
```

The Stage 2 pipeline is likewise disabled by default:

```bash
ros2 launch risabot_v4_experimental stage2_road_mask.launch.py enabled:=true
ros2 topic echo /v4_experimental/road/status
```

Stages 3 and 4 have separate disabled-by-default launches:

```bash
ros2 launch risabot_v4_experimental stage3_pose.launch.py enabled:=true
ros2 topic echo /v4_experimental/pose/status

ros2 launch risabot_v4_experimental stage3_uwb_bridge.launch.py enabled:=true
ros2 topic echo /v4_experimental/uwb/status

ros2 launch risabot_v4_experimental stage4_trajectory.launch.py enabled:=true
ros2 topic echo /v4_experimental/trajectory/status
```

Stage 4 will report its physical-validation blockers and produce no candidates
with the checked-in configuration.

Stage 5 is also disabled by default:

```bash
ros2 launch risabot_v4_experimental stage5_parking.launch.py enabled:=true
ros2 topic echo /v4_experimental/parking/status
ros2 topic echo /v4_experimental/parking/proposed_path
```

It reports the missing calibration, geometry, rear-coverage, and goal-source
evidence instead of inventing a parking target.

Stage 6 is independently disabled as well:

```bash
ros2 launch risabot_v4_experimental stage6_recovery.launch.py enabled:=true
ros2 topic echo /v4_experimental/recovery/status
ros2 topic echo /v4_experimental/recovery/proposed_path
```

With checked-in settings it reports calibration, policy-source, road, body,
rear-coverage, and LiDAR blockers and emits no proposal.

The contract sources and Stage 7 have separate disabled launches:

```bash
ros2 launch risabot_v4_experimental stage5_goal_source.launch.py enabled:=true
ros2 launch risabot_v4_experimental stage6_request_source.launch.py enabled:=true
ros2 launch risabot_v4_experimental stage7_arbitration.launch.py enabled:=true
ros2 topic echo /v4_experimental/arbitration/status
```

These launches expose blockers and diagnostic state. Checked-in gates prevent
them from producing a promotable request.

## Simulator relationship

The simulator is a design and validation reference. JavaScript simulator code
must not be copied into a live control path without ROS interfaces, timing,
freshness checks, recorded-data tests and hardware validation.
