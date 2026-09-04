# RISA-bot Gazebo Simulation — Implementation Spec

**Objective:** Build a Gazebo (Ignition) simulation of RISA-bot on a virtual replica of the
NXGV competition track, so that the *existing, unmodified* ROS2 nodes
(`heading_fusion`, `line_follower_camera`, `obstruction_avoidance`, `auto_driver`,
`parking_controller`) can be validated end-to-end before spending physical track time.

**Deadline:** Results needed by **Sept 15, 2026** (~11 days from spec date). Reserve the
final 2–3 days for physical-track-only validation — sim is a risk-reduction step, not a
replacement for the real gate.

---

## 0. Ground Rules / Non-Goals

- **Do NOT attempt to simulate the STM32/Arduino firmware.** The real MCU (Yahboom
  Rosmaster board) handles low-level motor/servo control over UART and is out of scope.
  The simulation boundary is `/cmd_vel` — Gazebo's Ackermann plugin consumes it directly
  and moves the simulated chassis. This is intentional: it keeps the sim focused on the
  ROS2-level logic (perception, planning, avoidance, fusion) where recent development and
  bug fixes have concentrated.
- **Before writing any sensor/topic code, grep the actual repo** for the real topic
  names and message types each node subscribes to / publishes. Do not assume names like
  `/imu/rpy` or `/scan` are exactly right — confirm against:
  - `heading_fusion.py` (subscriptions to IMU + odom, publications of fused heading/odom)
  - `line_follower_camera.py` (camera topic + message type it subscribes to)
  - `obstruction_avoidance.py` (LiDAR topic + message type)
  - `parking_controller.py` (LiDAR topic + message type)
  - `bringup.launch.py` (node startup order, delays, any existing remaps)
  - `params.yaml` (all tunable thresholds — reuse the same file for sim if possible,
    so tuning stays centralized)
- If any node expects a topic/type that Gazebo's standard plugins don't produce natively
  (e.g. a custom `/imu/rpy` Float32 array instead of standard `sensor_msgs/Imu`), write a
  small converter node rather than modifying the real node's interface.

---

## 1. Reference Specs (from project history — verify exact current values in repo)

### Hardware
| Component | Spec |
|---|---|
| Compute | RDK X5 SBC (companion computer, runs ROS2 Humble) |
| Motor control | Yahboom Rosmaster STM32 MCU, UART `/dev/myserial`, 50Hz |
| Drive | Single rear-wheel drive motor + encoder (no front drive) |
| Steering | Front Ackermann servo (CH4), `auto_right_steer_boost: 1.3` asymmetry correction |
| Camera | Astra camera (RGB, used for lane scanline detection) |
| LiDAR | YDLiDAR Tmini Plus, mounted at z=+0.12m, physically reversed (0° ray points to rear bumper) |
| IMU | Onboard 6-axis (MPU9250/ICM20948 class), publishes raw rpy/gyro |
| `lidar_angle_offset` | 3.1416 rad (180°) — hardware mount correction, standardized across all nodes |

### Track (verify exact current dimensions in repo/competition rules before building)
| Element | Spec |
|---|---|
| Track footprint | ~6.4m × 4m |
| Lane corridor width | ~40cm (tight — VFH+ hysteresis tuned for this) |
| Parallel parking slot | 0.75m (length) × 0.40m (width) |
| Perpendicular parking slot | 0.40m (width) × 0.40m (depth) |
| Features | Tunnel section, roundabout, obstacle zone (dodge), parking zone |

### Known Node Behaviors to Replicate in Sim
- `line_follower_camera.py`: fits 2nd-degree polynomial `x = ay² + by + c` across N=8
  scanlines; NaN/occlusion guards fall back to last-known-good curve with time decay;
  curvature evaluated exactly at endpoints + analytical inflection point
  `y* = -b/(2a)` (guarded by `abs(a) > 1e-6`).
- `obstruction_avoidance.py`: VFH+ over a ±45–50° forward LiDAR arc, 3-point/Gaussian
  smoothing `[0.2, 0.6, 0.2]`, 15% directional hysteresis band, cubic Bezier dodge with
  apex clearance scaled by 4/3 to guarantee ≥15cm actual clearance, degenerate-case stop
  if obstacle <0.20m.
- `heading_fusion.py`: complementary filter using message `header.stamp` for exact Δt
  (not loop-rate assumption), stationary deadband (~0.2°/s) to prevent drift, continuous
  angle wraparound across ±180°, bicycle-model velocity (`v_chassis = v_wheel_rear`, no
  cosine correction — rear-wheel encoder is already axis-aligned).
- `parking_controller.py`: closed-loop LiDAR wall-distance feedback, `lidar_stop_dist:
  0.15m` e-stop trigger, separate from maneuver travel distances and slot dimensions
  (keep these three concepts distinct in the sim world too — don't conflate slot size
  with stop-trigger distance when placing parking geometry).
- `auto_driver.py`: curvature-adaptive speed — `v_target = v_forward * max(0.4, 1.0 /
  (1.0 + 2.5*|κ|))`. Event-driven transitions (roundabout exit, tunnel exit at LiDAR wall
  divergence >0.85m) with bounded timeout fallback `EventDetected OR ElapsedTime ≥ T_max`.

---

## 2. Phase Breakdown

### Phase 1 — Gazebo + ros2_control Skeleton (Day 1–2)
- New ROS2 package, e.g. `risabot_sim`, alongside the existing `risabot_automode` package.
- Install/confirm `gazebo_ros2_control` and an Ackermann steering controller
  (`ackermann_steering_controller` or equivalent) compatible with ROS2 Humble + your
  Gazebo version.
- Build a minimal URDF/Xacro for RISA-bot: chassis footprint, wheelbase, track width,
  steering joint limits matching the real Ackermann geometry. Approximate mass/inertia
  is fine initially.
- Get an empty flat-ground world running with the car responding to `/cmd_vel`
  (`geometry_msgs/Twist`: `linear.x`, `angular.z`) before adding any sensors.
- **Acceptance criteria:** publishing to `/cmd_vel` manually (e.g. `ros2 topic pub`)
  visibly drives the simulated car forward and turns it in Gazebo.

### Phase 2 — Simulated Sensors Matching Real Topics (Day 2–3)
- **Camera:** Gazebo camera plugin publishing on the exact topic/type
  `line_follower_camera.py` subscribes to (verify in code — likely
  `sensor_msgs/Image` on something like `/camera/image_raw` or `/astra/color/image_raw`).
- **LiDAR:** Gazebo ray/GPU-ray sensor plugin publishing `sensor_msgs/LaserScan` on the
  topic `obstruction_avoidance.py` / `parking_controller.py` subscribe to (likely
  `/scan`). Match angular resolution/range as closely as practical to the YDLiDAR Tmini
  Plus specs. **Apply the same 180° mount orientation** in the sensor's URDF joint (or
  equivalently in a static transform) so the existing `lidar_angle_offset` param behaves
  identically in sim and reality — don't "fix" the mount virtually, replicate the real
  physical reversal so the same offset param is exercised.
- **IMU:** Gazebo IMU plugin. If `heading_fusion.py` expects a custom format
  (e.g. `/imu/rpy` as Float32 rather than standard `sensor_msgs/Imu`), write a thin
  converter node rather than changing the real node.
- **Odometry:** Either use Gazebo's built-in odometry publisher from the Ackermann
  plugin, or (more faithful) publish raw wheel-tick-style data matching whatever
  `/odom` message the real single-rear-encoder setup produces, so `heading_fusion`'s
  bicycle-model velocity logic is genuinely exercised rather than bypassed.
- **Acceptance criteria:** `ros2 topic echo` on each simulated sensor topic shows
  plausible data matching the expected message type; `heading_fusion` node (launched
  against sim) publishes non-NaN `/fused_heading` without code changes.

### Phase 3 — Track World (Day 3–6, likely the longest phase)
- Model the ~6.4×4m track: 40cm corridor walls, tunnel section, roundabout, an
  obstacle-dodge zone, and parallel + perpendicular parking slots at their real
  dimensions (0.75×0.40m and 0.40×0.40m respectively).
- **Critical for the camera pipeline:** the lane markings need color/contrast that
  the real `line_follower_camera.py`'s detection logic (HSV thresholding or similar —
  check the actual method in code) will actually pick up. Match the real track's line
  color and approximate lighting; a visually-plausible-but-wrong-hue line will make sim
  results not transfer to the physical track. This is the phase most worth spending
  real time on, since it directly determines whether Phase 4–5 results are meaningful.
- Add obstacle models with realistic LiDAR-visible geometry and placement matching
  competition rules (both centered and off-center placements, to also verify LiDAR
  angle/mirroring correctness — see Section 3 below).
- **Acceptance criteria:** a manually-teleoperated run through the world produces
  camera images where the existing color-threshold logic (tested in isolation, e.g. a
  quick script) correctly identifies lane boundaries, and LiDAR scans correctly show
  obstacle/wall geometry at expected ranges.

### Phase 3.5 — Blocking Prerequisite: ROS Time Migration
- **Critical Fix Required:** Almost all existing nodes (`auto_driver.py`, `dashboard.py`, `health_monitor.py`, etc.) rely on `time.monotonic()` for state transitions and loop metrics. If launched against Gazebo with `use_sim_time=True` at any real-time factor other than 1.0, timeouts (like bounded event fallbacks) will completely desync.
- **Action:** Before launching nodes against the sim, migrate all `time.monotonic()` calls to `self.get_clock().now().nanoseconds / 1e9` across the entire codebase. Do not proceed to Phase 4 without this fix, or validation scenarios in Phase 5 will produce misleading results.

### Phase 4 — Launch Real Nodes Against Sim (Day 6–7)
- Create `sim_bringup.launch.py` (copy of the real `bringup.launch.py` with sim-specific
  topic remaps only if names differ — ideally zero remaps needed if Phase 2 matched
  topics exactly).
- Bring up `heading_fusion`, `line_follower_camera`, `obstruction_avoidance`,
  `auto_driver`, `parking_controller` unmodified, pointed at sim.
- **Acceptance criteria:** the simulated car autonomously follows the lane without
  manual teleop input — this is the point where "does the real code work" becomes
  testable at all.

### Phase 5 — Validation Against the Same Physical Gates (Day 7–9)
- Low-speed baseline lap (match real `forward_speed: 0.15 m/s`) — confirm lane lock,
  sign/landmark triggers, event-driven transitions.
- Full-speed 2-lap continuous test (`forward_speed: 0.22–0.28 m/s`) with live
  curvature-based speed regulation active.
- Specifically probe the failure classes already guarded against in code, to confirm
  the guards actually trigger correctly in a live closed loop (not just unit tests):
  - Occluded/noisy scanlines → polynomial fallback to last-known-good curve
  - Obstacle near track edge → VFH+ hysteresis doesn't oscillate
  - Obstacle very close (<0.20m) → Bezier degenerate stop fires instead of a bad curve
  - Tightening curve just past the lookahead midpoint → inflection-point curvature
    sampling correctly slows the car down early (this was the most recent math fix —
    worth a dedicated sim scenario with a parabola whose peak isn't at the sample
    midpoint)
  - Parking approach → closed-loop LiDAR depth stop at the correct threshold, distinct
    from slot dimensions

### Phase 6 — Regression Suite (ongoing through Day 9)
- Record a `rosbag2` for every scenario where a bug is found and fixed.
- Build a simple replay script that runs all saved bags through the current node
  versions and flags any behavioral regression, mirroring how `test_path_planning.py`
  catches math regressions today.

### Phase 7 — Physical Track Only (Day 9–11, reserved buffer)
- Sim will not catch real sensor noise characteristics, servo/motor backlash, wheel
  slip, or real-world lighting variation. Do not treat sim validation as a substitute
  for the physical gate — only as a filter that reduces how many physical attempts are
  needed to reach a working state.

---

## 3. Specific Things to Verify/Test in Sim (carried over from prior review)

- **LiDAR mirroring:** confirm the sensor's angle convention (CW vs CCW as viewed from
  above) matches what `θ_body = θ_sensor + lidar_angle_offset` assumes. Test with an
  **off-center** obstacle (not directly ahead) and confirm the avoidance/dashboard
  direction matches the true side — a symmetric/centered test obstacle won't reveal a
  mirrored-axis bug.
- **Per-node offset defaults:** if any node still has a hardcoded fallback value for
  `lidar_angle_offset` (or similar params) separate from `params.yaml`, the sim is a
  good place to catch this by deliberately testing with a node that hasn't been synced.
- **Bezier apex clearance:** verify in sim (not just unit test) that the ≥15cm clearance
  actually holds through a full closed-loop dodge maneuver at speed, not just at the
  single geometric apex point in isolation.

---

## 4. Directory Structure Suggestion

```
src/
  risabot_automode/        # existing — unmodified
  risabot_sim/              # new package
    urdf/
      risabot.urdf.xacro
    worlds/
      nxgv_track.world (or .sdf)
    launch/
      sim_bringup.launch.py
      gazebo_world.launch.py
    config/
      sim_params.yaml       # if any sim-only tuning needed; prefer reusing real params.yaml
    scripts/
      imu_converter.py      # only if message format conversion is needed
```

---

## 5. Definition of Done

- [ ] Simulated car drives autonomously around the full track (both laps) using
      unmodified competition nodes.
- [ ] All five known failure-guard behaviors (Section "Phase 5") are triggered at least
      once in sim and confirmed to fire correctly.
- [ ] At least one off-center obstacle test confirms LiDAR angle/mirroring correctness.
- [ ] Regression bag suite exists and passes against current code.
- [ ] Sim results documented (what passed, what didn't) before physical track time is
      spent, so physical attempts are spent confirming fixes rather than discovering
      new bugs from scratch.
