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

Stage 3 mirrors `/odom` as the local pose and filters valid `/uwb_fix`
measurements into a separate coarse-global rigid transform. A UWB update
cannot rewrite the local pose. Innovation gating rejects gross outliers, and an
odometry discontinuity clears the coarse offset. UWB fusion remains gated until
the UWB and odometry frame alignment has been measured.

Stage 4 generates nine short bicycle-model rollouts and checks the complete
vehicle footprint against the synchronized road mask plus fresh LiDAR points.
It publishes JSON candidate diagnostics only. Processing remains blocked until
camera calibration, road thresholds, body geometry, minimum turning radius,
and LiDAR extrinsics have each been measured and explicitly marked validated.

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

ros2 launch risabot_v4_experimental stage4_trajectory.launch.py enabled:=true
ros2 topic echo /v4_experimental/trajectory/status
```

Stage 4 will report its physical-validation blockers and produce no candidates
with the checked-in configuration.

## Simulator relationship

The simulator is a design and validation reference. JavaScript simulator code
must not be copied into a live control path without ROS interfaces, timing,
freshness checks, recorded-data tests and hardware validation.
