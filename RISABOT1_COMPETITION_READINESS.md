# Risabot 1 competition readiness (2026-09-23)

Source: `Stage 2 _ Driverless CarBot Challenge 26_27 Guideline, Rules and Regulations.pdf` in this repository. This is a check of the active `track_test.launch.py` path on Risabot 1, not a claim that a feature elsewhere in the repository is active or physically verified.

## Current decision

The installed board stack is a supervised lane/tunnel test, not a complete competition mission. It starts in MANUAL. A full-lap AUTO run should wait for the lane boundary fix and mission integration below. Three manual reference bags are useful camera/road examples, but manual steering is not a lane-following target: outside parking, prerecorded movement is not allowed by the rules.

The measured dark strip is approximately 0.31 m between the inner white-line edges. Outside-to-outside tire width was reported as 0.205 m. The ideal static clearance is only `(0.31 - 0.205) / 2 = 0.0525 m` per side before steering and measurement error. The supplied rules cap vehicle width at 0.20 m (p. 2); remeasure the widest part with wheels straight and correct the physical car if the 0.205 m figure is confirmed.

## Rule-to-runtime coverage

| Requirement in supplied rules | Active Risabot 1 path | Work and evidence required |
| --- | --- | --- |
| Stay on the dark driving strip without touching lane boundaries; autonomous recovery (pp. 11-12) | Camera/BEV/mask/V4 steering active, but not reliable at corners | Evaluate and execute the same steering command. Compare the full predicted tire footprint with observed white edges, using one-edge recovery when the other leaves the camera view. Verify on held-out bag frames and a short physical corner. |
| Smooth lane change left (p. 5) | No active lane-change state or route-aware V4 path | Detect the relevant lane geometry, request the left branch, and verify that the executed V4 path follows it. |
| Roundabout exit selected from boom gate: open to tunnel, closed to parking (p. 5) | Legacy mission has gate-based route selection, but current lane-only V4 launch bypasses it; V4 steering does not consume `/lane_route` | Connect the gate state and selected route to V4 corridor/path selection and test both exits. |
| Curved, dark tunnel (p. 6) | LiDAR tunnel follower launched on Risabot 1 | Check centered clearance through the actual tunnel and transitions in/out. |
| Stop for lowered gate, proceed when raised (p. 7) | Gate detector not launched in active track test; lane-only auto driver bypasses gate hold | Launch and validate gate detection, mission hold, and proceed only on fresh open observation. |
| Hill climb, controlled descent, traction (p. 7) | Hill duty boost exists, physical performance unverified | Verify climb and descent in AUTO; speed/odometry data must be repaired before relying on encoder-based control. |
| Detect speed-bump sign and slow before crossing (p. 8) | Sign class exists but is debug-only; no active speed-bump behavior | Publish the detection, trigger a pre-bump speed profile, and verify timing and crossing. |
| Red stop and green go without timed waits (pp. 9, 16) | Signage/traffic-light detector not launched in active track test; lane-only driver bypasses light hold | Integrate live light state and verify red/green transitions. Do not use elapsed-time-only release. |
| Detect parking sign, parallel or perpendicular parking, stop inside bay (pp. 9-10) | Legacy parking playback exists; active V4 lane-only launch does not start sign or parking controller | Select the detected kind, validate recordings specifically on Risabot 1 or implement closed-loop parking, and verify final position. Recorded motion is permitted only for parking (p. 18). |
| No manual intervention during scored challenge (pp. 12, 18) | Controller takeover works for testing | Competition run must be fully autonomous; any testing takeover is a failed challenge, not a passing run. |

## Lane-controller evidence from the three manual bags

The three saved Risabot 1 bags contain 1,424 selected trajectory frames. In 762, the selected path had at least one road-mask-blocked rollout step; in 309, all 55 steps were blocked, yet it could still be marked `valid`. In 183 frames the evaluated candidate's first steering command and the command later reported for lane control had opposite signs. These were manual laps, so this does not prove physical boundary contact or autonomous actuator behavior; it proves the present planner validity signal cannot be used as tire-clearance evidence. `track_test_config.py` disables road-support enforcement, while `trajectory_shadow.py` substitutes a separate filtered-centerline command after candidate evaluation. Simply enabling mask rejection would cause frequent stops because the current corner masks can shrink to a wedge.

Across 1,512 saved road-status frames, plausible both-edge mask rows measured a median dark run width of about 0.28 m at 0.8-1.1 m ahead, and about 0.275 m at 1.1-1.4 m. This is narrower than the approximately 0.31 m physical strip. The mask is therefore not a calibrated representation of the tire-clearance boundary. The center of a clean two-edge run remains useful, but one-edge reconstruction needs a measured mask-edge offset or an independent white-line detector rather than treating the mask edge as the physical white-line edge.

The correction is to derive left/right white-edge geometry with confidence from the calibrated BEV, infer the center from one edge and measured strip width only for a short bounded interval, and score the *actual command to be sent* with a steering-lag rollout against the tire footprint. Slow for curvature and low clearance, preserve a short IMU-assisted turn through transient occlusion, and stop when neither camera geometry nor a suitable tunnel wall observation can bound the path. Use the manual bags for independent visual/clearance benchmarks rather than imitation of the joystick commands.

## Recorded Risabot 1 AUTO corner, 2026-09-23

The valid bag is on the board at `/home/sunrise/track_test_run/trials/20260922T220112Z`. It captured 19.72 seconds of AUTO followed by MANUAL and a zero command. The user observed the tires cross the right white line. This is a failed corner test, not a completed lap.

The fitted lane error swung from -0.067 m to +0.268 m in about three seconds. The original steering command was about -0.31 rad at the start, reversed only after the error had crossed zero, then reached +0.59 rad as the car approached the opposite line. The previous 0.8 rad/s steering-rate cap alone requires about 0.6 seconds to reverse from -0.23 to +0.25 rad. The Risabot 1 test tune reduces the cross-track gain from 1.40 to 0.75 and raises the steering-rate cap to 2.0 rad/s; this tune still needs a short physical retest.

The camera and mask frames show a separate failure near the roundabout marking. The connected dark-road mask shrank from 14 corridor rows to zero near 10.6 seconds, and the car briefly received a zero drive command. A naive vertical morphological bridge connected the near fragment to a large far dark region, but its confirmed white edges existed only around 1.6 m ahead and the resulting fitted curve was implausible. That bridge was not deployed. Continuing through this gap requires a route-aware roundabout model or another validated near-field observation, not simply removing the stop.

One attempted full-lap recording used the wrong ROS domain and captured no sensor messages. `record_autonomy_test.py` now fails fast if it receives no MANUAL/drive topics and can remain active through manual repositioning with `--continuous`. Set `ROS_DOMAIN_ID=1` on Risabot 1 and wait for its `recording_ready` response before asking the operator to drive. Do not use the invalid recording at `/home/sunrise/track_test_run/trials/20260922T215435Z` as evidence.

## Minimal verification order

1. Verify camera edge and center estimates on held-out straight/corner/tunnel bag frames, including known mask failures. Check predicted tire-edge clearance and command-to-rollout identity.
2. Check physical steering sign, delay, and attainable left/right radius on Risabot 1 once. Repair encoder feedback before using travel distance for mission transitions.
3. Run one short straight and one short left/right corner under supervised AUTO, then a tunnel and roundabout exit test. Record camera, mask, inferred edges, intended path, selected command, and actual actuator command.
4. Integrate and test gate, light, bump-sign, hill/descent, and both parking variants, then a complete uninterrupted autonomous course. Do not mark any challenge passing from node presence or manual recordings alone.
