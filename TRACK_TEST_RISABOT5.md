# Risabot 5: front-camera lane test

Current readiness and deployment note (2026-09-23 Malaysia time): the geometry
lane controller, adaptive 65%-to-48% motor duty, proportional manual throttle,
IMU plan hold, and observed-boundary continuation are deployed on Risabot 5.
Two manual reference laps were recorded and replayed; the cleaner lap showed
82.2% turn-direction agreement between manual steering and the camera-derived
centerline. The isolated replay exercised 100 low-footprint-support frames using
live observed boundaries instead of decaying toward straight. The board service
starts in MANUAL and the latest project-owned regression is 368 passed, one skipped.
Use [RISABOT5_AUTONOMY_TODO.md](RISABOT5_AUTONOMY_TODO.md) for the next test sequence
and [the connection audit](AUTONOMY_CONNECTION_AUDIT_2026-09-22.md) for full-mission
gaps. The board observations and earlier 306-test result below are historical.

This mode tests lane centering only. It runs the front camera, V4 BEV, road mask,
trajectory, arbitration, motion executor, command controller, servo controller,
joystick and dashboard. It does not run the complete competition mission.

## Settings

- `vehicle:=risabot5`: measured servo center **80**. `risabot1` selects **110**.
- `motor_duty:=65`: straight and turning requests are **65% motor duty**, not
  0.65 m/s. The existing command conversion is 255 duty units per speed-request
  unit, so the internal request is `65/255`. This does not measure physical speed.
  Startup acceleration is time-limited; autonomous output is also capped at 65
  in the servo bridge. Manual-controller speed selection is separate.
- `steering_gain:=2.8`: correction gain is applied within the predicted trajectory,
  with a 0.22 m lookahead. The configured 50-degree actuator range is
  available; the old unmeasured 0.4 m turning-radius restriction is removed in
  this mode. Servo travel and wheel-angle mapping still need physical verification.
- The car-1-only right-turn boost is not applied to this car-5 test.
- Steering-dependent speed reduction keeps 65% duty on straights and no less
  than 48% during strong corrections. Low swept-footprint support does not make
  the car straighten when at least half of usable corridor rows still contain
  an observed white boundary; genuine visual loss falls back to the IMU-held plan.
- Commissioning flags, signage prerequisites, scan point-count requirements,
  and the 98% road-footprint rejection do not block this test mode. Observed
  fresh LiDAR obstacles still reject intersecting trajectories. Stale/missing
  optional LiDAR is ignored; it is not treated as measured clearance.
- Camera calibration, current lane data, command freshness, e-stop, manual
  takeover, joystick watchdog, finite commands and actuator bounds remain active.
  Starting the launch starts in MANUAL; it does not arm AUTO.

Validation flags remain honest: test mode bypasses the commissioning checks
without changing unmeasured items to `validated: true`. Ordinary competition
launch behavior remains separate.

## Run on the board

The board source packages are directly under `~/risabotcar_ws/src/`.
The local Windows repository stores them under its `src/` directory as well.
Source changes require a board rebuild; some installed packages are copies.

```bash
source /opt/tros/humble/setup.bash
source ~/risabotcar_ws/install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0
cd ~/risabotcar_ws
colcon build --packages-select control_servo risabot_automode \
  risabot_v4_experimental risabot_v4_control --executor sequential
source install/setup.bash

ros2 launch risabot_v4_control track_test.launch.py \
  vehicle:=risabot5 motor_duty:=65 steering_gain:=2.8 \
  minimum_turn_duty:=48 steering_slowdown_gain:=0.85 \
  profile_path:=/home/sunrise/risabot5_profiles/camera_profiles.yaml
```

The reviewed profile is installed at the path above and in the experimental
package. It uses the actual native 320x240 capture. The Astra's offset lens does
not require moving the centered camera housing. Held-out board error was
2.230 mm RMS / 4.855 mm maximum; two additional printed guide marks had a maximum
error of 7.509 mm. These are local sheet checks, not a full-track lens validation.
The secondary camera remains uncalibrated. Recalibrate if the camera mount moves.

Start only one camera/control stack. If a camera driver is already intentionally
running at the profile's exact resolution, add `start_camera:=false`.
`start_lidar:=false` omits the LiDAR driver. `dashboard:=false` omits the dashboard.
The launch sets the camera resolution from the selected profile, keeps color at
15 Hz, and disables unused depth, IR and point clouds.

Open `http://192.168.137.74:8080/` and use V4 Road for synchronized BEV/mask views.
The first joystick button press unlocks input and is consumed; release with
sticks neutral. The current mapping toggles AUTO/MANUAL with Start (index 11)
or Y (index 4). Confirm that mapping on the connected controller. Selecting
AUTO is the action that starts the track test. The manual gear display is not
the configured autonomous motor duty.

Useful observations:

```bash
ros2 topic echo /v4_control/status
ros2 topic echo /v4_experimental/trajectory/status
ros2 topic echo /cmd_safety_status
```

`/v4_control/status` reports `track_test_mode`, requested motor duty, selected
source and hold reason. `/cmd_safety_status` reports its hold reason and final
normalized steering. Positive steering means RIGHT; negative means LEFT.

## Verification completed locally

- Final project-owned regression suite: 368 passed, one optional test skipped.
- The real node classes, with inert ROS/hardware interfaces, carry a lane
  proposal through arbitration, command limiting and the servo bridge to exactly
  65% duty. Missing camera, stale command, e-stop and non-lane states produce zero.
- Reverse-to-zero motor requests now apply zero immediately. The removed
  per-callback ramp previously changed a -100 to zero request into -90.
- A simulated straight lane with initial offsets of +/-0.10 m converged to
  approximately +/-0.0005 m after 1.79 m using the new steering configuration.
  This is a bicycle-model check, not evidence of physical track performance.

Camera fit evidence and remaining physical measurements are documented in
`RISABOT5_CALIBRATION.md` and `calibration_risabot5/`.

## Board deployment verified on 2026-09-22

All four changed ROS packages rebuilt successfully. All 19 deployed files were
checked against their installed copies by SHA-256, including the five-panel
dashboard and the reviewed camera profile. The board clock had reset to 2000;
after correcting it, source files were recopied to refresh modification times
so setuptools could not silently retain newer-dated old build files.

Source/install backup: `/home/sunrise/track_test_backups/20260922_100840/before.tar.gz`.
Prior board-local camera profiles are in that directory's `camera_profile_before/`.
Runtime log: `/home/sunrise/track_test_run/full_stack/launch.log`.
The live dashboard provides BEV, coverage, candidate, connected and fused tiles
from matching source timestamps. Select **V4 Road** in the camera view.

After the final rebuild, an eight-second read-only observation saw 109 camera
frames, 33 road-status updates, no BEV/road errors, MANUAL mode throughout, and
zero raw/final autonomous drive commands. The dashboard delivered five live
composite frames with all five stages synchronized. At that observation, LiDAR
returns intersected every proposed trajectory and no gamepad or `/joy` messages
were present. A stationary track check, controller connection, and physical
steering/centering test remain to be performed; no autonomous drive was started.
