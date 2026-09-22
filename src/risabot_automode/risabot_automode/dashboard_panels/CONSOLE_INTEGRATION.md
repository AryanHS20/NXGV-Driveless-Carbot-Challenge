# Console remodel: inventory and integration decision

Status: **incomplete integration preview**, not the completed functional remodel.
The target is the original NXGV feature checkout, as agreed in
`DASHBOARD_V4_HANDOFF.md`. The separate V2 checkout is reference only.
No running dashboard or robot process was modified or deployed.

## Current local implementation (2026-09-22)

- Opt-in `/console` page; the existing `/` dashboard and camera behavior remain
  unchanged. Layout uses the supplied mockup CSS, with all 24 section identities
  and all 138 section-button definitions preserved in a tested inventory.
  This is not yet visual or functional parity with the complete mockup: most
  calibration panels still expose requirements rather than measured workflows.
- Navigation, split panes, missing-backend explanations, real existing status
  snapshots when hosted by ROS, and explicit absent/stale receive ages.
- Draft venue-lap import, whole-map fit/refit, constrained handle dragging,
  per-section fit reports, map export, P0–P3 editing/rotation/flip/snap, isolated
  three-leg planning and fingerprinted map/mission draft-bundle export.
- One fixed computation subprocess, bounded input/output/logs, 90-second timeout,
  cancellation and revision checks. No shell commands, ROS process supervision,
  driver restarts or actuator commands are issued by the new code.
- No automatic activation: draft bundles are stored in
  `$XDG_STATE_HOME/risabot/dashboard-drafts`, or
  `~/.local/state/risabot/dashboard-drafts` if unset. Each export gets a unique
  directory. Existing exports and central parameters are not overwritten.
- Imported lap coordinates are operator-provided, **not** proof of a calibrated
  venue frame. Vehicle geometry remains the supplied ZIP geometry, not measured
  hardware. Neither local nor coarse V4 poses are recorded as venue coordinates.
- Launch switching, actual motor authority, camera capture/calibration, versioned
  calibration sessions/rollback, tuning, live lap recording, and runtime mission
  activation remain open. In particular, the preview's E-STOP does not stop the
  car; this is stated prominently and physical controls are required.
- Quick point-pair realignment and automatic per-control-point nudging are not
  implemented by the supplied helpers; they are not falsely advertised as done.

### Preview without ROS or a robot connection

From `src/risabot_automode`, run:

```powershell
python -m risabot_automode.dashboard_panels.console_preview --port 8766
```

Open `http://127.0.0.1:8766/console`. It binds loopback only. The preview uses
real planning algorithms but no invented live sensor values. Its Existing
dashboard link is useful only when `/console` is hosted by the ROS dashboard.

### Verification so far

- 89 relevant dashboard/planning/API tests passed, including subprocess crash,
  timeout/cancel/retry, unknown motion actions rejected, malformed/oversized
  requests, stale browser revisions, invalid inputs, exact fingerprints,
  incomplete coverage, and preserved control inventory.
- Browser checked missing-lap errors, launch-blocking guidance and map rendering.
- JavaScript syntax checked. Existing dashboard goldens still pass unchanged.
- Not complete: real hardware, full-route success, calibrated recording,
  downstream activation and all-button runtime acceptance/stress tests.

## Inputs preserved

- Repository `carbot_gui_mockup.html`: 24 rendered sections, 138 section
  button instances, and 28 shell/navigation button instances in its initial
  race state. Calibration navigation is generated dynamically as well.
- `console_control_inventory.json` records all section buttons and input
  controls, including controls disabled in the mockup. Disabled sample states
  must become backend-derived reasons, not permanently dead controls.
- `Setup Carbot.zip`: `map_builder.py` and `mission_planner.py` copied here.
  Only the planner import was adapted for package use; both remain runnable
  directly. Neither is imported by the production dashboard yet.
- Supplied `track_map.yaml` and `mission.yaml` remain in the original ZIP;
  they are examples, not automatically activated vehicle configuration.

## Two different installed architectures

| Concern | Existing NXGV checkout / risabotcar_ws | NxGV-Carbot-Autonomous-V2 |
| --- | --- | --- |
| Current track test | `risabot_v4_control track_test.launch.py` | Separate, inactive workspace |
| HTTP backend | `risabot_automode.dashboard`, `/data`, `/camera_feed` | `carbot_gui.gui_server`, `/api/core`, `/api/tab/<name>`, `/api/img/<key>` |
| Calibration/race launches | No matching pair in this checkout | `carbot_bringup calibrate.launch.py` and `race.launch.py` |
| Wizard | No 13-step wizard | `carbot_ops.calibration_wizard` exists but is a stub: all actions refused |
| Race START | Existing controller/joystick authority; no matching race service | GUI calls `/carbot/race/start` and observes preflight/armed state |
| Mission v2 consumer | No `mission.yaml` loader in the current source tree | `carbot_common.mission`, global planner, mission logic, race supervisor |
| Source on board | `/home/sunrise/risabotcar_ws` | `/home/sunrise/NxGV-Carbot-Autonomous-V2` |

These facts were checked from source, including read-only SSH. The current
track-test task owns deployment, rebuilds, control/calibration configuration,
and robot lifecycle. New console work must remain local during that test.

The mockup matches the V2 architecture. Selecting the existing V4 checkout
requires implementing missing mission, race-supervisor, calibration, and
launch contracts in addition to a UI remodel. Selecting V2 requires a local
editable copy of that source and integration with its existing contracts.
The target is now the existing feature checkout; missing backend integrations
remain assigned to the control/integration task rather than copied from V2.

## Required behavior by workflow

| Workflow | Backend work and verification |
| --- | --- |
| Header, START, manual, hand back, E-STOP, STOP MOTORS | Use acknowledged authority services and observed state. Never switch modes only in the browser. Retain the takeover confirmation dialog. |
| Calibrate/race switch | Supervisor survives launch shutdown. Stop conflicting jobs, wait for owned children to exit, check readiness, then start the selected launch. Do not kill processes owned by the track-test task. |
| Sensor check, camera restart | Real rates/ages and failures; restart only managed camera jobs with owner and conflict checks. |
| Camera identity | Live feed selection, recorded mapping, role swap, validated session save. No invented sensor identity. |
| Intrinsics | Per-camera capture/delete/auto-capture, board coverage, compute worker, measured result, save; use actual tool thresholds. |
| Extrinsics/IPM | Actual target layout and measured corners; compute worker, preview and failure guidance; never use sample PASS values. |
| LiDAR alignment | Recorded camera/LiDAR observations, measured alignment, review before session save. |
| IMU/odometry | IMU request acknowledgement; distance/spin/drift capture stages with explicit start/finish controls. |
| Steering limits | Bounded calibration commands, stop control and observed response; no unattended automatic search for mechanical binding. |
| Speed PID/practice | Explicit operator-run bounded jobs, prerequisite checks, continuously available stop, measured result. UI tests must use inert transports. |
| Lighting | Road/line ROI selection, real sample distribution, threshold validation, session overlay consumed by perception. |
| UWB | Editable measured anchor positions, named-point capture, offset computation and separate verification. |
| Map builder | Record real fused venue poses without map feedback; stop recording before fitting. Reuse ZIP `fit_rigid`, `move_handle`, `section_report`, and `to_yaml_dict`. |
| Mission planner | Reuse ZIP road/body checks, lane/bay snapping, rear-axle poses, flip, rotation, leg search, reverse pieces, exits and `mission_dict`. Replan affected legs after pose changes. |
| Save/redo/keep/rollback | Timestamped sessions and real pass/fail. Only keep previously valid results with matching dependencies. Atomic files; rollback only while conflicting jobs are stopped. |
| Tuning | Real parameter catalogue, validation, acknowledged set/get, revert and session persistence; enforce race restrictions on server. |
| Diagnostics and split view | Preserve every tab, filter, layer and selector. Poll only visible demand; show absent/stale values explicitly; cap image, grid, trail and event payloads. |

## Map/mission downstream contract

1. `track_map.yaml` v2 contains track-frame geometry and track-to-venue
   transform. The recorded lap stays in the venue frame.
2. `mission.yaml` v2 stores rear-axle poses, per-leg route pieces, reverse
   directions, parking bay handoffs, roundabout exits, and SHA-1 of the exact
   map bytes. A changed map invalidates the mission, including after rollback.
3. The V2 mission loader additionally requires `mission_rules.yaml` for speed
   zones, traffic/gate rules and leg end behavior. That file is not in the
   supplied ZIP; the selected architecture's existing rules must be retained
   and validated against all exported legs.
4. Race readiness must validate the entire map/mission/rules/calibration set
   and verify downstream consumers loaded that same revision. A successful
   disk write alone is not successful activation or permission to drive.
5. The map-builder source explicitly leaves automatic per-control-point
   nudging unfinished. Preserve its implemented whole-map fit and manual
   handle editing; do not advertise the unfinished refinement as working.
6. The mockup has 13 steps, including a separate mission-planner step. The
   available V2 calibration definition has 12 and ends with practice runs.
   Session compatibility and required-step validation must be updated together.

## Failure and stress-test acceptance

- Exhaustively enumerate controls against the inventory; every operation has
  either an implemented action or a visible concrete prerequisite/recovery
  explanation. An unimplemented action is not described as an impossible one.
- One calibration/planning worker per resource; return a conflict with the
  running job name and Stop action. No unbounded job queue or shell input.
- Failed process, nonzero exit, timeout, hung worker and cancel leave the
  console serving HTTP. Exit result and bounded logs remain visible; Retry is
  available after the owned process is stopped.
- Sensor disconnect, stale images, malformed payloads, missing nodes and
  network loss clear live claims. Reconnect must not replay action requests.
- Race/calibration switch requests are serialized. Partial start and failed
  stop have explicit states; no overlapping motor/camera owners.
- Test revision races: map changes during mission planning, old worker
  completion, concurrent save, rollback, and stale backend responses.
- Test small screens, split view, hidden-tab throttling, browser reload,
  server restart, and polling failures with an inert backend before hardware.

## Preparation checks

- Original map-builder demo runs headlessly with its synthetic lap and reports
  approximately 2.4 mm translation error and 0.012 degree heading error.
- New compatibility tests cover map geometry round-trip, exact fingerprint,
  frame transforms, constrained handles, body-preserving flip, off-road
  rejection and incomplete mission serialization.
- These checks are not a completed UI stress test or live integration test.
