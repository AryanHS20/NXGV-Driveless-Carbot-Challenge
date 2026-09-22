# V4 validation — strict-first paint allowance and reverse-and-rejoin recovery

## What changed

The previous planner could reject all nine steering rollouts. It had no escape
manoeuvre. It could also hold when the planned lane-change opening was outside
usable local camera evidence. A screenshot alone does not identify which gate
caused a particular stop; V4 reports separate road, camera-paint and obstacle
rejection reasons in the candidate chips.

V4 evaluates the strict road boundary first. Only if no strict candidate is
feasible does it evaluate nine more candidates using the selected 0–5 cm paint
allowance. It preserves the real vehicle dimensions and the rendered road.
Planning retains 5 mm body padding. Original-boundary crossings are still
assessed; allowing a path in the simulator does not make it competition-legal.
Physical obstacles are not covered by the paint allowance.

After a recoverable planning problem persists for 0.45 s, recovery brakes and
searches Reeds–Shepp connections to up to five future mission-route poses.
Candidates must start in reverse, end forward, use exactly one gear change,
plan no more than the configured reverse distance (default 20 cm), have total
length <=1.6 m, fit the allowed road envelope and avoid current LiDAR obstacles.
Recent camera/memory evidence must support at least 55% of sampled rear road.
These heuristic thresholds are simulation settings, not physical guarantees.

Execution is limited to 3 cm/s, includes stopped steering alignment and a stopped
gear change, and checks the upcoming path against current obstacle observations.
After rejoining, the normal lane planner resumes. If a forward path becomes
feasible while waiting, recovery cancels. Failed searches rescan every 2 seconds;
three attempted manoeuvres at the same location are the limit. This is bounded
analytic recovery, not a guarantee that every trapped pose is recoverable.

Emergency stop, motion/camera staleness, local uncertainty, unresolved route
identity, physical contact, gate/light holds and the command watchdog retain
priority. Recovery may reposition for missing *local branch visibility*, but
cannot override an unresolved *course-location conflict*.

## Full normal run with high UWB noise

Chromium/WebGL, direct file URL, seed 2026, known start, 20 cm UWB Gaussian sigma
per axis, default geometry, 5 cm strict-first allowance, recovery enabled:

- Both missions completed in **365.17 simulated seconds**.
- Original boundary / obstacle contact events: **0**.
- Parallel parking: **full body contained**, including the 8 mm assessment inset.
- Final heading error: **0.48 degrees**.
- Final rear-axle offset from the bay-centre goal: **2.98 cm**.
- Local navigation RMS error: **1.11 cm**.
- Minimum sampled corner margin: **1.69 mm**; not a physical safety allowance.
- No browser page errors.

Actual periodic logs and summary: `validation/v4-20cm-run.csv` and
`validation/v4-20cm-summary.json`. These are assessment outputs, never replay
inputs to the controller. V3's baseline/dropout results were not re-labelled as
V4 full-mission tests.

## Deliberately induced lane-change deadlock

The browser test starts at mission-2 route sample 509 (approximately x=3.853 m,
y=1.082 m) with a deliberate -0.5 rad heading perturbation, at rest. This is a
constructed failure case, not a claim to reproduce the user's unseen settings.
The car fits the strict road initially. Fresh camera rendering, road extraction,
memory and LiDAR run normally; perception is not replaced with a perfect mask.

- Recovery evaluated **72** actual analytic connections; **6** passed its checks.
- It selected approximately **7 cm planned reverse** followed by a forward merge.
- It changed gear at rest and completed recovery at **15.73 simulated seconds**.
- The following 24-second observation confirmed normal driving resumed.
- A subsequent crossing of the original boundary was recorded: approximately
  **0.43 cm** maximum sampled excursion during that short observation.
- There were **no physical obstacle contacts** in that observation. The line
  crossing is deliberately retained in assessment, not hidden by the allowance.

`recovery-short-check.json`, `recovery-search.png` and `rejoined-route.png`
contain the recorded short test. The full continuation then completed parking at **119.85 simulated seconds**
from the perturbed start, with one recovery, one original-boundary crossing and
zero physical obstacle contacts. Maximum sampled excursion remained **0.43 cm**.
See `recovery-continuation.json` and `recovered-and-parked.png`.

## Safety and interface checks

Browser assertions passed for emergency stop during recovery, camera failure
during recovery, the request watchdog during recovery, braking when automatic
recovery is disabled, and the live 0/5 cm allowance slider. No page errors.
See `validation/recovery-fault-checks.json`.

Portable `npm test` checks passed: all prior geometry/analytic endpoint/control
checks; camera/UWB separation and branch checks; exact paint-envelope allowance;
existence of a bounded reverse-first/forward-last connection for the constructed
deadlock; rejection without rear evidence; rejection of a LiDAR obstacle; and
preservation of emergency/camera holds during recovery.

## Limits

Crossing the original course boundary and reversing outside parking on a one-way
course are training overrides, not newly established competition permissions.
Set paint allowance to 0 and disable recovery to test strict forward-only driving.

The model still assumes a known start and a drawing-derived prior map. Local
vision is prior-map-aided; it is not general SLAM. UWB remains a separate coarse
estimate. Image cells are 1.8 cm; memory cells are 2.5 cm; uncertainty indicators
are approximate. Fresh sensor data and sampled checks do not guarantee that an
unobserved obstacle is absent. Recovery stops if no checked route is found.

Car size is still 30 × 19.2 cm, wheelbase 21.6 cm. Default minimum rear-axle radius
40 cm, camera mounts/FOV, overhang split and actuator dynamics remain provisional.
The original confirmed LiDAR mast height violation remains unresolved physically.
The collider does not certify mast/camera clearance, steering tyre sweep, tyre
friction, suspension, tipping or real braking. No real-car or RDK X5 test was run.
