# Carbot Simulator — V4

Extract the ZIP to a new folder, then double-click `index.html`.
No install, administrator access, terminal, ROS, server or internet is required.
`START_HERE.html` contains button-by-button instructions.

## Test the upstream parking controller

Open `index.html`, click **Parking script test**, and choose parallel or
perpendicular. Click **Play controller** or scrub the timeline. The yellow bay
uses the documented 0.75 m parallel slot dimension and 0.40 m perpendicular
width; its other dimension is an editable 0.40 m default assumption. The
phase logic comes from Aquadrox's
[`parking_controller.py`](https://github.com/Aquadrox-Technologies/NXGV-Driveless-Carbot-Challenge/blob/c6321539fee7961f2cf9023b6fe50f652eed59da/src/risabot_automode/risabot_automode/parking_controller.py)
(same file on upstream `main` and `refactor-test`). Change start X/Y, heading or
maximum wheel angle or bay depth to examine sensitivity. The readout shows the commanded
linear/steering values, active phase, odometry, heading and whether the whole
car footprint fits at the parking wait. The source commands and thresholds are
ported directly; 21 cm wheelbase, 27.5 × 18.5 cm body, ideal speed/odometry and
steering angle are simulator assumptions. A real parking pass is not established.

The upstream `refactor-test` sign path selects saved servo playback, so this
panel directly starts the phase controller for inspection. The separate R5
recording replay below it displays actual manual commands and does not execute
the upstream controller.

## R5 recorded parking replay

Open `index.html` and scroll to **R5 recorded parking replay**. Select parallel
or perpendicular, then Play, pause, step or scrub through the exact 20 Hz saved
motor PWM and steering commands. The chart is recorded data; the top-down path
is an illustrative bicycle-model sketch because the recordings contain no
measured pose or bay alignment. This panel is offline and never sends commands
to R5. The 3D mission above it remains a separate R1 simulation. Source SHA-256
values appear below the chart. To regenerate the embedded data, run
`python build_parking_data.py PARALLEL.json PERPENDICULAR.json` with the two
original saved R5 recordings.

Click **20 cm**, then **Run both missions**. Choose the map and local-planner
views. UWB now affects a separate coarse global estimate; camera boundaries
and odometry guide local motion. Cyan shows observed corridor centres; grey
shows evaluated candidates; blue shows the selected trajectory. Yellow is the
local navigation pose, violet the coarse global estimate, green the actual pose.

The Risabot 1 footprint now uses the rechecked 27.5 cm body length and 18.5 cm
outside tire width, with a 29.5 cm dark lane. The 21 cm wheelbase, 4.2 cm rear
overhang, and adjustable 40 cm rear-axle turning radius remain model inputs to
verify physically. Exactly one command owner applies the 200 ms watchdog.

This model uses a known start and a prior map. It is not map-free navigation,
a complete EKF/SLAM implementation, or a calibrated real-vehicle dynamics model.
The published run in `VALIDATION.md` used the earlier geometry and is not a
validation of this revised footprint. The simulator is a separate model; the
physical Risabot 1 controller uses its own calibrated camera and footprint.

Developers: `npm ci`, `node build.mjs`, and `npm test`.
Runtime: bundled Three.js 0.186.0 / Rapier 0.17.3; licences included.

V4 adds a live 0–5 cm paint-boundary allowance, strict-first candidate selection,
and automatic brake → short reverse → stopped gear change → forward rejoin
recovery. Both features start enabled. Physical obstacles and sensor/emergency
holds stay hard limits. Original-boundary crossings are counted separately from
physical contacts. These are training overrides, not rulebook amendments.

Read the new V4 section in START_HERE.html and current VALIDATION.md.

## Show the real Risabot 1 in 3D

On the laptop connected to the robot, run from this folder:

```powershell
python live_server.py --dashboard http://192.168.137.161:8080
```

Open `http://127.0.0.1:8765/` in the laptop browser. Click **Connect live car**.
With the physical car stationary at the marked START and facing toward the lane
change, click **Align at START**. The orange car and trail then follow fresh
hardware `/odom` reported by the robot dashboard. The simulator's synthetic
mission pauses. Disconnect to return to it.

This view is read-only. The orange pose is an odometry estimate, so it can drift
from the physical car, especially after wheel slip or an odometry reset. It is
not ground truth or UWB localization. The live mode hides the car when hardware
odometry is stale and requires alignment again after a large pose jump.
