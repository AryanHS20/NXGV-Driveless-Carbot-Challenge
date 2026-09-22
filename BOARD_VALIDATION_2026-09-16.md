# Dashboard and board validation — 16 September 2026

## Traffic-warning correction (later update)

- Advance warning signs set an advisory only; they no longer block lane following.
- A detected traffic lamp without confirmed colour publishes `unresolved`.
  Mission control holds this stop until confirmed green, including when the lamp
  subsequently disappears. Confirmed red/yellow also remains latched until green.
- Traffic waits display `TRAFFIC_LIGHT`; a real e-stop or sensor interlock retains
  `EMERGENCY_STOP` even when a red-light latch is also present.
- 59 local tests and preflight pass, including warning-only driving, uncertain
  lamp hold, red-to-unknown hold, green release and e-stop label priority.
- Three updated files: mission_logic.py, auto_driver.py, signage_detector.py.
  Board backup: `/home/sunrise/nxgv-traffic-backup-20260916-174016`.
- Stationary manual mode verified before installation. No motion or synthetic
  traffic-light topic commands used for verification. Track validation remains.

## Deployment

- Board: `sunrise@192.168.137.161`, workspace `/home/sunrise/risabotcar_ws`.
- Active repository is nested at `/home/sunrise/risabotcar_ws/src`, based on
  `d1511634336de23fc221eb60bb26149c87468589`. The outer workspace Git history is older.
- Before changes, 27 active application Python files and the installed parameter
  file matched the local feature branch. The configured BPU model hash passed.
- Applied fixes to local source and board source; no Git commit or push performed.
- Backups on the board:
  - `/home/sunrise/nxgv-dashboard-backup-20260916-165111`
  - `/home/sunrise/nxgv-performance-backup-20260916`
- Performance measurements used launch PID `19660`, log
  `/home/sunrise/bringup-dashboard-performance-20260916.log`. After correcting
  a source-transfer text encoding mismatch, final launch PID is `22794`, log
  `/home/sunrise/bringup-dashboard-final-20260916.log`.
- Verified manual mode, zero reported speed and idle playback before restarts.
  No driving, calibration or playback command was issued.

## Fixes

- Publish signage debug images even when inference finds zero detections.
  Traffic Light and Signage share this feed.
- Wait for an actual JPEG after view changes instead of spinning on an empty
  frame slot. Idle stream connections expire after five seconds and can reconnect.
- Keep the parameter helper's name independent of launch-wide node-name remapping.
- Use a single dashboard executor worker because callbacks share one mutually
  exclusive group; HTTP and parameter-service handling have separate threads.
- Combine health-monitor and dashboard stale-stream lists. Exclude the inactive
  odometry source and treat change-only fused-obstacle updates as events.
- Launch the camera factory with the existing board settings, omitting unused,
  unavailable point-cloud components. XML fallback disables point clouds too.
- Limit OpenCV worker pools to one thread per perception/dashboard process.
- Publish signage preview images at most 10 Hz and 320x240; inference inputs and
  detection thresholds remain unchanged.

## Verification

- 55 local unit tests pass, including zero-detection preview, preview size/rate,
  empty-view waiting, stream expiry and existing safety regressions.
- Preflight passed. ROS Humble build passed locally for `risabot_automode` and
  on the board for `risabot_automode` and `obstacle_avoidance_camera`.
- Final ROS graph includes `/dashboard` and `/dashboard_param_helper` separately,
  with all expected application nodes present. No component-load failures,
  tracebacks or process-death errors found in the final startup log.
- Live signage preview advanced. Twelve one-second samples during preview showed
  image ages 2–171 ms. Safety-loop averages were 48.91–50.40 Hz; auto-driver
  averages were 44.28–50.34 Hz, with 11 of 12 samples at least 48.89 Hz.
- Traffic-light preview also advanced after switching. This confirms transport
  recovery, not classification accuracy on competition signs.
- CPU remains heavily loaded: one final snapshot had 8.2% idle. Thermal readings
  remained around 90–92 degrees C during verification, versus about 95 before.
  Cooling and sustained performance remain unresolved; short stationary samples
  do not establish track reliability.

## Remaining work

- Check fan, heatsink and airflow physically; repeat timing checks after cooling.
- Validate camera/sign accuracy, stopping distances and behavior on the track.
- Both parking recording parameters remain empty. Startup found 20 historical
  recording files; an earlier empty dashboard list reflected missing state
  updates, not proof that the files were absent. Historical recordings have not
  been validated for this control contract and were not played.
- `dashboard_ctrl` remains missing in the displayed freshness list; controller
  telemetry and initial recording-state delivery need separate follow-up.
- This update is uncommitted locally and on the board; GitHub still contains
  the preceding feature-branch commit until the user commits and pushes.
