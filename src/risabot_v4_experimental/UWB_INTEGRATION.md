# V4 UWB integration

The UWB path is a diagnostic coarse-global position source. It never publishes
motion commands and it never rewrites local wheel/IMU odometry.

## Data path

`/uwb3/input_json` -> `uwb_bridge_shadow` -> `/v4_experimental/uwb/fix` -> `pose_shadow`

The bridge consumes each anchor's `sample_seq` once, clears its cache when the
tag `boot_id` changes, compensates measured range bias and antenna height, and
solves a robust range multilateration fix. It rejects stale ranges, impossible
height corrections, poor anchor geometry, and excessive residuals.

Valid fixes are published only after at least one new radio measurement. This
prevents downstream estimators from counting a repeated tag report as another
independent observation. A one-shot invalid fix is published when the valid fix
stream times out.

## Physical gates

All three gates in `config/uwb.yaml` ship false:

- `anchor_geometry_validated`: surveyed anchor x/y coordinates are verified.
- `range_offsets_validated`: per-anchor range offsets are measured and checked
  at a second location.
- `antenna_heights_validated`: anchor and tag antenna z values are measured.

The bridge cannot publish a valid fix until every gate is true. Stage 3 pose
fusion has a separate `uwb_frame_alignment_validated` gate which must stay false
until the UWB axes are measured against the odometry axes.

## Board validation order

```bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
source /opt/tros/humble/setup.bash
source ~/NXGV-Driveless-Carbot-Challenge/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic hz /uwb3/input_json
ros2 topic echo /uwb3/input_json --full-length
```

Confirm a cold tag restart produces a new `boot_id` and all three anchors within
five seconds. Then calibrate offsets, update `config/uwb.yaml`, and launch:

```bash
ros2 launch risabot_v4_experimental stage3_uwb_bridge.launch.py enabled:=true
ros2 topic echo /v4_experimental/uwb/status
ros2 topic echo /v4_experimental/uwb/fix
ros2 launch risabot_v4_experimental stage3_pose.launch.py enabled:=true
```

Validate the position at five or more surveyed locations, both stationary and
moving. Record median error, 95th percentile error, dropout duration, rejected
fix rate, and the worst observed jump. Keep the pose alignment gate closed until
a straight drive establishes the UWB-to-odometry yaw repeatedly.
