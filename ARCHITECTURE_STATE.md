# RISA-bot — Current Architecture & Issue Log

> The car as actually built (repo `main`), plus every known problem.
> Companion docs: `HANDOFF.md` (ops handoff), `Guide/competition_layout_overview.jpeg` (track),
> `Carbot Architecture V1.html` (long-term proposal — most of it is NOT built; § "Proposal vs reality" maps the gap).
> Last updated: 2026-09-12.

## 1. Hardware (actual)

- **Chassis:** Yahboom Rosmaster, Ackermann steering, wheelbase 0.14 m. Single rear-drive motor
  with encoder (code uses motor channel 0 only; channels 1–3 are undriven noise). Steering servo
  ID 4, center 100 units, −50/+70 range, ~50° lock → ~12 cm minimum turning radius.
- **Compute:** Horizon RDK X5 (ARM + BPU accelerator).
- **Camera:** Orbbec Astra Pro at **8.5 cm height, 0° tilt** — two USB functions: depth
  `2bc5:0403` (OpenNI) + color `2bc5:0501` (UVC). It mostly sees room, not road.
- **LiDAR:** YDLiDAR Tmini Plus over CP2102 serial, 230400 baud, 10 Hz, ±180°, 0.02–16 m.
- **Others:** IMU (roll/pitch/yaw JSON), USB gamepad, Rosmaster board over `/dev/myserial` (ch341) at 115200.
- **Not present:** no side cameras, no UWB anchors, no EKF — localisation is gyro + wheel encoders only.

## 2. Software: the real pipeline

- **Sensors:** vendor `astra_camera` driver (container + factory + UVC driver),
  `ydlidar_ros2_driver_node`, static TF, `joy_node`.
- **Perception:** `line_follower_camera` → `/lane_error`, `/lane_curvature`, `/lane_lost`
  (scanline or sliding-window mode, Kalman, polynomial + Pure Pursuit); `obstacle_avoidance`
  (LiDAR); `obstacle_avoidance_camera` (edge-based); `signage_detector` (YOLO11n on BPU →
  hill/parking/traffic-light/obstacle/tunnel-confidence); `boom_gate_detector` (LiDAR);
  `tunnel_wall_follower` (RANSAC wall fit → cmd_vel + `/tunnel_detected`); `heading_fusion`
  (gyro+odom complementary → `/fused_heading`); `obstruction_avoidance` (VFH + Bezier);
  `parking_controller` (timed phases); `health_monitor`; `dashboard` (HTTP :8080, MJPEG + JSON).
- **Brain:** `auto_driver` — one flat priority state machine
  (MANUAL > TRAFFIC_LIGHT > BOOM_GATE > OBSTRUCTION > TUNNEL > … > LANE_FOLLOW),
  PID steering + slew limiter + adaptive speed, 0.25 s dwell gate with immediate-state exemptions.
- **Actuation:** `cmd_safety_controller` (50 Hz caps, 0.35 s timeout) → `control_servo`
  (manual/auto toggle, asymmetric servo map with 1.3× right-turn boost, odometry publisher)
  → Rosmaster serial.
- **Structural fact:** there is **no trajectory anywhere** — perception emits an error scalar,
  the brain emits Twist directly. No corridors, no goal struct, no sampler, no manoeuvre
  solver; parking/obstruction run timed open-loop phases. Topic constants in
  `src/risabot_automode/risabot_automode/topics.py`; central params in
  `src/risabot_automode/config/params.yaml` (live-tunable, no rebuild).

## 3. Issues — lane perception

1. Scanline mode hunts paint row-by-row: dies on sharp corners (road goes parallel to rows),
   shadows, glare, faded tape. Sliding mode exists but was never tuned on a real surface.
2. Default `invert_binary: true` = dark-seeking on mixed floors (tiles, grout, mats, shadows
   all qualify) — observed locking onto texture at 7/8 "confidence."
3. IPM off by default with uncalibrated guess ratios; polynomial fit runs on as few as 3 points;
   `white_threshold` fixed at 100.
4. The 15 px/frame expected-shift clamp strands the tracker in fast corners; curvature enters
   only as a heuristic divisor — R_min is never enforced.

## 4. Issues — camera plumbing (all solved, all fragile)

5. Color needs three robot-side-only fixes (`use_uvc_camera: true`, vid/pid `0x2bc5/0x0501`)
   in an untracked backup file — a fresh checkout does not contain them.
6. Depth sensor repeatedly vanishes from USB (cable/port marginality); fixed by reseat so far, twice.
7. BPU runtime wedges on SIGKILL mid-inference (`dnn.load` hangs) — only a reboot clears it.
   Never kill −9 the stack casually.

## 5. Issues — signage / model

8. Class-ID churn across model versions; `nxgv_yolo11n_deploy_v2/` evaluated and REJECTED
   (compat aliases for a problem this repo doesn't have; would regress video + tunnel fixes).
9. Tunnel vision decoupled to advisory `/tunnel_confidence` — correct, but tunnel entry is now
   LiDAR-only. If LiDAR misses the tunnel mouth, the car sails past.

## 6. Issues — localisation (weakest layer after lane)

10. No UWB/EKF — heading is gyro corrected by *commanded* steering (verified in code: no
    steering sensor exists), so backlash and servo lag enter yaw unobserved; position is raw
    encoder dead reckoning with `ticks_per_meter` and odom scales still at defaults
    (likely uncalibrated).
11. IMU needs manual calibration; pitch sat at 7° vs the 8° hill trigger in one session.

## 7. Issues — planning / control

12. Single flat state machine — can't represent combined situations (roundabout + red light).
13. Double control loop: the lane node already outputs Pure-Pursuit steering, then `auto_driver`
    PIDs it again with kp=0.8 plus a slew cap — redundant, attenuating, laggy into hairpins.
14. Steering asymmetry handled by a 1.3× fudge factor instead of a measured pulse→angle table;
    `servo_center 100` vs mechanical 90.
15. Parking/obstruction are timed open-loop phases — slip-sensitive, no replanning on drift.
16. Observed 1 Hz `LANE_FOLLOW ↔ REVERSE_ADJUST` judder loop (triggered by near-blind LiDAR
    episode, not by dwell logic).
17. `traffic_light_detector.py` is dead code (signage does TL now); pointcloud plugin errors are
    cosmetic; duplicate `/dashboard` discovery echoes are harmless.

## 8. Issues — systems / ops

18. Three-repo workspace mess on the robot (upstream template at root, our code nested at `src/`,
    driver in a third repo) — pulling in the wrong directory silently deploys stale code.
19. Symlink-install + wiped sources = dangling-link cascade; credentials live in gitignored
    scratch scripts (never commit); mDNS flaky (use the IP); ICS link marginal (63–298 ms
    dashboard latency).
20. No rosbag baselines, no lap metrics, gains still at day-one defaults, traffic-light dataset
    still uncaptured (0/400 images).

## 9. Proposal (`Carbot Architecture V1.html`) vs reality

Yes — the full 14-section proposal was read end to end. Mapping its asks onto this car:

- **Do now, cheap:** raise the 15 px shift clamp; lock camera exposure/white balance; build the
  measured pulse→angle calibration table (§12); tune lookahead gains live.
- **Do soon, medium:** corridor-style region perception (§03) or at minimum tune the sliding
  tracker; curvature feedforward / single-loop steering (§09–§11 overlap with our open item).
- **Do not build (yet):** side cameras, UWB + EKF (§05), prior map + mission planner (§07),
  sampler (§09), manoeuvre solver (§10) — all assume hardware/time we don't have; revisit only
  after the tuned baseline's traces prove the ceiling.
- **Structural critiques accepted:** single flat state machine can't do roundabout+red (§08);
  no trajectory/goal struct anywhere (§09); reflex must not steer (§12 — ours complies).

## 10. What blocks the competition date, in order

1. LiDAR health proof (dense scan, not 9 points) — everything safety-related hangs on it.
2. Lane polarity + mode decision on the replica, then one disciplined tuning day (gains → slew → speed).
3. Dataset capture (needs the working camera + track time, ~1 hour total).
4. Curvature feedforward — only if tuned-baseline traces show roundabout lag/hairpin widening;
   design it from that data.

## 11. Official reference video (SIMULATED — not real footage)

File: `WhatsApp Video 2026-09-15 at 6.25.42 AM.mp4` (21 MB, 640×360 @ 20 fps, ~3.6 min).
Issued by the competition organizers. Carries a "SIMULATED DATA" badge throughout: synthetic
onboard POV + track minimap + telemetry overlays, ~20 scripted chapters (track entry → 90°
corners → traffic light red→green → tunnel with gate arm → lane change → roundabout →
parallel park → L park → MISSION COMPLETE).

What it's worth to us:

1. **Venue look, confirmed:** dark asphalt, WHITE solid edge borders, WHITE dashed center line,
   green surroundings. This settles the polarity debate for the real track — white-seeking
   (`invert_binary: false`) is correct there. Lab mats differ, so treat lab tuning as
   approximate and expect re-tuning at the venue.
2. **Challenge order + visuals:** matches our state machine sequence; the traffic light is a
   clearly visible red lamp on a pole, tunnel has a red/white gate arm, parking bays are marked.
   Good mental model for what each detector must face.
3. **Reference speeds:** the sim drives 0.12–0.18 m/s with steering up to ~36° — our
   `forward_speed 0.15` sits right in that band, so our speed scale is sane.
4. **Their approach:** border tracking (both edges + center fit) — essentially what our sliding
   tracker does. Validating, not news.

What it is NOT good for: **training images.** Do not mix these synthetic frames into the real
dataset — the rendered-vs-camera domain gap will poison the model. The 400 real images from
the car are still needed. The video's real uses are venue familiarization and HSV sanity-checks
(the rendered red lamp is a decent reference for the color-vote logic).
