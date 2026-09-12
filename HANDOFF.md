# RISA-bot — AI Handoff Brief

> Read this entire file before touching the robot or the code.
> Last updated: 2026-09-12. Status: robot healthy, all streams live, tuning in progress.

## 1. Project

- **What:** Autonomous RC car (Yahboom Rosmaster chassis) for the NXGV Driveless competition.
- **Repo:** `https://github.com/AryanHS20/NXGV-Driveless-Carbot-Challenge`, branch `main` (this file lives at repo root).
- **Stack:** ROS 2 Humble, Python nodes, Horizon RDK X5 BPU for sign detection.
- **Hardware:** RDK X5 board, Orbbec Astra Pro (depth `2bc5:0403` via OpenNI + color `2bc5:0501` via UVC), YDLiDAR Tmini Plus (`/dev` CP2102), Yahboom Rosmaster motor/servo board (serial, udev `/dev/myserial` → ch341), USB gamepad receiver, 2.4G joystick.
- **Track:** 6.4×4 m loop, 0.4 m lanes. Layout image: `Guide/competition_layout_overview.jpeg`. Challenges: 1 Obstruction, 2 Roundabout (full circle), 3 Tunnel, 4 Boom gate, 5 Hill, 6 Bumper, 7 Traffic light, 8 Parallel park, 9 Perpendicular park. A full replica is built in the lab — all testing happens there.

## 2. Workspace layout (read carefully — it is messy)

**Local dev machine:** `C:\Users\Aryan\OneDrive\Desktop\Competition\NXGV-Driveless-Carbot-Challenge\` (this repo).

**Robot (`risabot1`):**
- `~/risabotcar_ws/` is a git repo tracking the **upstream template** (`eemrull/RISA-bot`). **DO NOT `git pull` there** — you will merge upstream code over ours.
- Our code lives in `~/risabotcar_ws/src/`, which is a **separate git repo** tracking **this repo** via remote `aryan`, branch `main`, upstream pinned to `aryan/main`. Update with:
  ```bash
  cd ~/risabotcar_ws/src && git pull aryan main
  cd ~/risabotcar_ws && colcon build --symlink-install --packages-select risabot_automode
  ```
- Because the repo root sits at `src/`, ROS packages resolve to nested paths, e.g. `~/risabotcar_ws/src/src/risabot_automode/`. This is normal here.
- `src/ros2_astra_camera/` on the robot is a **third repo** (driver source, restored from a local backup — it is NOT fully tracked in this GitHub repo; only a skeleton exists under `src/ros2_astra_camera/` here). Its `astra_mini_params.yaml` + `astra_mini.launch.py` carry robot-side fixes (UVC vid/pid, `use_uvc_camera`, see §6) with `.bak_*` files next to them. Never delete `src/ros2_astra_camera` — there is no remote copy.
- Old `scratch/*.py` SSH helper scripts on the dev machine are **gitignored** (they contain the robot password in plaintext — never commit them).

## 3. Architecture

- Launch: `ros2 launch risabot_automode bringup.launch.py` (all nodes; `competition.launch.py` is the lean variant). ~20 nodes.
- **Perception:** `line_follower_camera` → `/lane_error`, `/lane_curvature`, `/lane_lost`; `signage_detector` (YOLO11n BPU) → hill/parking/traffic-light/obstacle/tunnel topics; `obstacle_avoidance` (LiDAR) + `obstacle_avoidance_camera`; `tunnel_wall_follower`; `heading_fusion` → `/fused_heading`.
- **Brain:** `auto_driver` — state machine (LANE_FOLLOW, OBSTRUCTION, ROUNDABOUT, TUNNEL, BOOM_GATE, TRAFFIC_LIGHT, HILL, PARK×2, REVERSE_ADJUST…) + PID steering (`_lane_follow_cmd`).
- **Actuation:** `cmd_safety_controller` (limits) → `control_servo` → Rosmaster serial 115200.
- **UI:** `dashboard` node serves `http://<robot>:8080` (MJPEG + JSON). Topic constants: `src/risabot_automode/risabot_automode/topics.py`. Central params: `src/risabot_automode/config/params.yaml` (all nodes tunable live from the Parameters drawer, no rebuild).

## 4. Robot access

- **Host:** `192.168.137.161:22` (Windows ICS link from the dev laptop), user `sunrise`.
- **Password: NOT in this repo** (would leak to GitHub). Owner holds it; local untracked `scratch/*.py` scripts use paramiko from the dev machine.
- **Dashboard:** `http://192.168.137.161:8080/` — prefer IP over `risabot1.local` (Windows mDNS is flaky). Hard-refresh (Ctrl+F5) after any dashboard change; browsers cache the page aggressively.
- **SSH rules learned the hard way:**
  - Never put `pkill -f <pattern>` and the literal launch text in one shell command — pkill matches your own shell and kills it mid-chain. Split kills / verify / launch into separate SSH calls, and use bracket-shielded patterns (`[b]ringup`).
  - Kill orderly (TERM, wait, KILL stragglers), `rm -f /dev/shm/sem.astra_device_sem`, launch exactly ONE bringup. Overlapping launches wedge USB/BPU state.
  - Short-timeout `ros2 topic hz` / `node list` produce false negatives on this loaded board — use ≥20 s timeouts before concluding anything is dead.

## 5. Key parameters (all live-tunable)

- `auto_driver`: `forward_speed 0.15`, `pid_kp 0.8 / ki 0.01 / kd 0.20`, `speed_error_scale 1.5`, `min_turn_speed 0.4`, `lane_steer_slew 3.0`, `heading_gain 0.5`.
- `line_follower_camera`: `tracker_mode: scanline|sliding` (default scanline), `invert_binary: true` (dark-lane!), `use_otsu: false`, `ipm_enabled: false`, `n_scanlines 8`, Kalman on. Sliding adds `s_thresh / l_thresh / sobel_thresh / sliding_windows / margin / minpix` and forces IPM.
- `signage_detector`: YOLO11n, `model_path /home/sunrise/nxgv_yolo11n_640x640_nv12.bin`, per-class `thresh_*`, `tunnel_publish_enabled: true` (edge-triggered — see §7).
- `servo_controller`: `servo_center 100`, ranges L50/R70, `auto_right_steer_boost 1.3`, `steering_max_deg 50`, `wheel_base 0.14`.
- `dashboard`: `cam_encode_max_hz 10.0` (MJPEG cap).

## 6. Camera gotchas (solved — do not regress)

- Astra Pro color does NOT come through OpenNI — only via the UVC driver, which requires **both** `use_uvc_camera: true` (else the driver object is never constructed; zero `uvc` lines in log) **and** `uvc_camera.vid/pid: 0x2bc5/0x0501` in `astra_mini_params.yaml` (robot-side file).
- "Resource busy" on `2bc5:0403` = stale holder → orderly kill + sem cleanup + single relaunch.
- Depth sensor repeatedly vanished from `lsusb` — marginal USB cable/port. Currently on a working port; if `0403` disappears again, reseat/swap cable first (no software can fix an absent device).
- `verify_nxgv_pyeasydnn.py` (in the v1 drop folder, untracked) is the working BPU smoke test — the shipped `verify_nxgv.py` needs `hbm_runtime`, which the robot lacks.

## 7. Work log (what's done, what's open)

**Done:** BOOM_GATE/TRAFFIC_LIGHT logic enabled; subsumption obstacle on; params synced; `heading_fusion` in launch + feedforward into PID; feature-flagged sliding-window tracker (HLS+Sobel, prior+histogram, single-line fallback); YOLO11n swap (same topics); dashboard 10 Hz MJPEG throttle + fetch-based self-reconnecting video player; lane debug view cropped to detection region; `.bin` committed under `tools/bpu_model/model_output/`; `tools/capture_dataset.py` for retraining captures. Merged `fix/obstruction-tunnel-heading-fusion` (PR #1): OBSTRUCTION bypasses state-dwell debounce (safety preemption); `/tunnel_detected` solely owned by tunnel_wall_follower with signage tunnel vision moved to advisory `/tunnel_confidence` (edge-triggered); heading_fusion complementary filter now blends gyro+odom (alpha, dt-scaled).
**Evaluated and REJECTED:** `nxgv_yolo11n_deploy_v2/` (untracked drop folder) — same model hash, same params; its only additions are YOLOv5-compat param aliases (nothing here uses the old names) and an older dashboard copy that would regress the video fixes.
**Open / next:** (1) LiDAR health — returned only ~9 points during one session; verify dome clear + dense scan, else hardware fault. (2) IMU calibrate on level ground (pitch sat at 7° vs 8° hill threshold). (3) Lane polarity/mode comparison on the replica (`invert_binary:false`+Otsu vs `sliding`+IPM). (4) PID/slew/speed tuning per `Guide/tuning_guide.md`, one variable at a time. (5) Traffic-light dataset: 4×100 images via `capture_dataset.py` (none captured yet). (6) Decide curvature-feedforward steering only AFTER tuned-baseline data exists.

## 8. Quick commands (robot)

```bash
source /opt/ros/humble/setup.bash && source ~/risabotcar_ws/install/setup.bash
ros2 launch risabot_automode bringup.launch.py            # bringup (log: ~/bringup.log)
ros2 param set /line_follower_camera tracker_mode sliding # try new tracker (back: scanline)
python3 capture_dataset.py --label green --count 100      # dataset capture (in tools/)
lsusb | grep -i orbbec                                    # both 0403 + 0501 must appear
timeout 25 ros2 topic hz /camera/color/image_raw --window 3
```
