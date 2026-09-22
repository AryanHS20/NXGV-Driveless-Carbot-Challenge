# Risabot 1 reference-lap audit — 2026-09-23

The car completed three operator-driven reference laps in MANUAL. The bags closed cleanly and remain on `risabot1` under `/home/sunrise/track_test_run/manual_reference_laps/`. The incomplete first attempt was deleted at the operator's request. The lane stack was running from `feat/safety-contract-rework` at `b2afdac`. No autonomous-controller tuning was deployed during these laps.

| Lap | Recording directory (UTC suffix) | Bag duration | Messages | Camera frames | LiDAR scans |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 | `20260922T194013Z` | 105.98 s | 41,812 | 1,581 | 1,060 |
| 2 | `20260922T194329Z` | 96.04 s | 35,966 | 1,432 | 953 |
| 3 | `20260922T194547Z` | 96.16 s | 35,173 | 1,433 | 962 |

The preceding 4.10 s AUTO trial is under `/home/sunrise/track_test_run/trials/20260922T192924Z`. The operator observed a drift to the right and returned to MANUAL. During the first second, AUTO requested a small right steer (`+0.075` normalized) while the estimated center was only 1.5 cm right. The estimated center then moved 9–17 cm left. The AUTO left-steer request grew only after the drift, and the camera lost one boundary by the second second. The planned boundary clearance was negative thereafter. This supports testing earlier left correction and a deadband for small initial center estimates; it does not establish the mechanical steering center or wheel-angle gain.

## Sensor findings

- **Tunnel LiDAR:** Each lap had a sustained interval with close returns on both sides: 50.77–56.16 s, 43.10–47.99 s, and 41.90–45.80 s relative to each bag. The third interval had 40 consecutive scans, 84.5% median valid returns, and median raw side-sector 10th-percentile ranges of 0.227 m and 0.279 m. The `track_test.launch.py` starts the LiDAR driver, but does not start `tunnel_wall_follower`; the current lane-only controller therefore does not use wall centering. The LiDAR's physical left/right orientation still needs an alignment check because the existing tunnel follower applies a 180-degree angle offset.
- **Tunnel camera:** At the approximately aligned tunnel intervals, both lane boundaries were observed at the 0.65 m lookahead in only 7.4%, 0%, and 11.1% of road-status samples. Median inferred lane widths fell to 0.125, 0.135, and 0.165 m, versus the configured 0.32 m expected width. The fused road mask's median nonzero area was only 1–2%. Camera-only tunnel steering is therefore poorly supported by these recordings. The scan and event clocks are aligned approximately (within a few tenths of a second), so these percentages describe the tunnel passage rather than a precise entry/exit boundary.
- **Hill IMU and power:** The uphill pitch exceeded +5 degrees in every lap, peaking at 17.0, 17.5, and 18.6 degrees. Median manual motor duty during these uphill intervals was approximately 74%, 90%, and 94%. Downhill pitch then reached -15.0, -14.5, and -11.7 degrees. The current AUTO track-test cap is 65%, so it cannot request the power the operator used on two of the three climbs. The data does not prove the minimum duty needed to climb; it supports a pitch-triggered uphill power allowance and a controlled on-car check.
- **Encoder/odometry:** While manual forward command exceeded 30%, reported odometry speed was below 0.03 m/s for 93.8%, 95.3%, and 96.3% of sampled periods in the three laps. The controller log repeatedly filtered about 18–21 implausible encoder jumps per second during motion. The rear motor's separate encoder cable is reportedly connected. Inspect its pinout, channel, connector, and raw counts before using this odometry for distance, progress, or speed control. The current V4 trajectory node does not subscribe to `/odom`, so this fault alone does not explain the 4-second lane drift; the V4 pose estimate and any distance-based mission logic remain untrustworthy.
- **Speed bumper:** These bags contain camera, LiDAR, IMU roll/pitch/yaw, joystick and commands, but no reliable wheel-speed feedback or annotated bumper timestamp. No distinct IMU pitch interval above 3 degrees appeared outside the hill. The present evidence cannot establish why AUTO sometimes stalls on the bumper. A short annotated on-car pass and valid motor feedback would distinguish insufficient duty from a physical snag or perception stop.

The existing `tools/analyze_manual_reference_lap.py` found normal centerline support in 61–65% of driving samples and observed-boundary fallback in 33–36%. Its manual-versus-planner steering-direction agreement was 72–79% on samples with meaningful steering. This is a diagnostic comparison, not an accuracy score: the operator can anticipate turns and sometimes correct past mistakes.

## Next implementation and validation

1. Keep `risabot1` in MANUAL while checking the encoder cable channel/pinout and raw stationary/moving counts. Do not fix these jumps by merely raising the software jump threshold.
2. Replay the saved LiDAR scans through the existing tunnel follower offline, verify the 180-degree mount correction, and compare its wall-center command with the operator's steering. Integrate it into the track-test launch only after that sign and timing check.
3. Replay the camera and operator commands around each corner. Reduce steering delay and avoid initial wrong-way commands, then compare the new trajectory against the same bags before another AUTO run.
4. Add an uphill power profile keyed to sustained positive IMU pitch. Preserve proportional manual throttle and test the required AUTO duty on the real hill. Treat the speed bumper separately after a marked pass.

The bags are the replay benchmark. They are intentionally not committed to Git because each is roughly 0.7 GiB; the recording directories above are the source data on `risabot1`.
