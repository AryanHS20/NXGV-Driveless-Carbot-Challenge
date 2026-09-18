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

## Simulator relationship

The simulator is a design and validation reference. JavaScript simulator code
must not be copied into a live control path without ROS interfaces, timing,
freshness checks, recorded-data tests and hardware validation.
