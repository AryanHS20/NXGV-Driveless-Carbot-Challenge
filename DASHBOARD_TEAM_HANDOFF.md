# RISA Bot dashboard teammate handoff

Date: 2026-09-22 (MYT)

Repository: `https://github.com/AryanHS20/NXGV-Driveless-Carbot-Challenge.git`

Branch: `feat/safety-contract-rework`
Working checkout: `C:\Users\Aryan\OneDrive\Desktop\Competition\NXGV-Driveless-Carbot-Challenge`

This document describes the dashboard work that was prepared locally and then
committed to the branch above. It also states what is **not** finished. A new
developer should read this file, `DASHBOARD_V4_HANDOFF.md`, and
`src/risabot_automode/risabot_automode/dashboard_panels/CONSOLE_INTEGRATION.md`
before changing behavior.

## 1. Target and ownership

The target is the existing V4 feature checkout named above. The separate local
`NxGV-Carbot-Autonomous-V2` checkout was inspected only as architectural
reference and is not the deployment target for this dashboard work.

Dashboard-owned paths are:

- `src/risabot_automode/risabot_automode/dashboard.py`
- `src/risabot_automode/risabot_automode/dashboard_panels/**`
- `tests/test_dashboard*.py`
- `tests/golden_dashboard.html` and `tests/golden_teach.html`

The control/integration task owns launch files, central parameters, calibration
runtime adapters, mission/control/pose code, deployment and physical tests.
Do not mix those files into a dashboard-only change without coordination.

No part of this dashboard handoff was deployed to the robot, no ROS processes
were restarted, and no motion command was issued.

## 2. What is implemented and functional

### 2.1 Existing production dashboard: synchronized V4 Road composite

The existing dashboard at `/` remains the production interface. Its V4 Road
view was repaired for track-test use:

- The five tiles (BEV, coverage, candidate, connected and fused) are grouped by
  the exact ROS source timestamp. A newer partial cycle never reuses an older
  tile to appear complete.
- A 150 ms synchronization grace period allows the same source cycle to arrive.
  After that, missing tiles are visibly labelled as missing.
- A set older than 1.0 s is replaced by explicit stale placeholders; old pixels
  are not shown as live data.
- Only three timestamp sets are retained, bounding memory use.
- Encoding is demand-driven: it happens only for the V4 Road view, forward
  source, and an active camera client, using the existing 5 fps cap.
- Switching camera source or view clears cached JPEG and V4 tile state. Debug
  processed views force the forward camera; side-camera raw behavior and the
  renewable side-camera demand lease are preserved.
- Each tile is letterboxed to preserve aspect ratio and has a readable label.
  The five-wide strip scrolls on smaller screens instead of squeezing imagery.
- Invalid/unstamped images and conversion failures are rejected. Diagnostics
  are useful but rate-limited to one warning per key every five seconds.
- AUTO/MANUAL heartbeat receipt still refreshes freshness, but unchanged
  heartbeats no longer flood the log with false mode transitions.

Primary code:

- `dashboard.py`: subscriptions, timestamp-set cache, freshness/watchdog,
  placeholders, letterboxing, encoding and diagnostics.
- `dashboard_panels/routes_camera.py`: source/view switch semantics, clearing
  obsolete frames and side-camera demand publication.
- `dashboard_panels/center/camera/card.html`: V4 Road selector and five-tile
  presentation.
- `dashboard_panels/script/camera.js`: view switching and stream recreation.
- `dashboard_panels/script/navigation.js` and `shell/head.html`: layout behavior.
- `tests/test_dashboard_camera.py`, `tests/test_dashboard_panels.py`, and
  `tests/golden_dashboard.html`: behavior and rendered-output coverage.

### 2.2 Remodel groundwork: opt-in `/console` preview

The supplied `carbot_gui_mockup.html` was inventoried. Its 24 screens, 138
screen-button instances and 28 initial shell-button instances are retained as a
tested control catalogue. The mockup's fake success values are not copied into
live status.

The remodel is available as a separate `/console` route so it cannot silently
replace the existing dashboard. It currently provides:

- all tab identities and calibration steps 1 through 13;
- split-pane navigation;
- explicit red explanations for missing runtime adapters;
- real `/data` observations and receive ages when hosted by the ROS dashboard;
- live values cleared on disconnect instead of leaving stale claims visible;
- bounded polling, hidden-tab pause and no automatic replay of actions;
- a local map-builder and mission-planner workflow described below.

The prominent `E-STOP · NOT CONNECTED` and `STOP MOTORS · NOT CONNECTED`
controls are intentionally honest. They do not issue commands. The physical
controller/E-stop remains required until the control task supplies an
acknowledged authority adapter.

Main preview files:

- `dashboard_panels/shell/console.html`: remodel shell and styles.
- `dashboard_panels/script/console.js`: tabs, split view, polling, error display,
  SVG map/mission interaction and API calls.
- `dashboard_panels/console_control_inventory.json`: original DOM inventory.
- `dashboard_panels/console_inventory.py`: installed-package copy of inventory.
- `dashboard_panels/console_contract.py`: per-button availability/action/reason.
- `dashboard_panels/routes_console.py`: HTTP endpoints and bounded JSON handling.
- `dashboard_panels/console_preview.py`: loopback-only preview without ROS.
- `dashboard_panels/registry.py`: `/console` and `/api/console/*` routing.

### 2.3 Supplied Setup Carbot map builder and mission planner

The user's supplied algorithms were copied into the package:

- `dashboard_panels/map_builder.py`
- `dashboard_panels/mission_planner.py`

The map builder is otherwise the supplied implementation. The mission planner's
import was adapted so it works both as a package module and as a direct script.
Implemented web/backend semantics reuse its real functions:

- venue-frame lap import, with finite-number, frame, movement and 6000-point
  limits;
- whole-map rigid fit/refit through `fit_rigid`;
- constrained handle edits through `move_handle`;
- per-section coverage/RMS validation through `section_report`;
- exact `track_map.yaml` generation and SHA-1 fingerprint;
- rear-axle pose/body fit checks, lane/bay snapping, rotation, flip and reset;
- three bounded route searches through `plan_leg`;
- reverse route pieces, section/roundabout metadata and `mission.yaml` export.

Expensive work runs in one isolated, low-priority subprocess. It has bounded
input/output/logs, a 90-second timeout, cancellation, conflict messages and
revision guards. A stale worker result cannot overwrite a newer edit.

Files:

- `dashboard_panels/console_jobs.py`: single-job owner, timeout, cancel and logs.
- `dashboard_panels/console_worker.py`: fixed fit/mission subprocess; no shell.
- `dashboard_panels/console_planning.py`: thread-safe draft state and export.

Exports are drafts under
`$XDG_STATE_HOME/risabot/dashboard-drafts/<unique-id>` or, when that variable is
unset, `~/.local/state/risabot/dashboard-drafts/<unique-id>`. They do not modify
central parameters or create an `ACTIVE` marker. The manifest explicitly states
`activated: false`.

## 3. HTTP interface added

| Method and path | Purpose |
| --- | --- |
| `GET /console` | Serve the remodel preview. |
| `GET /api/console/catalog` | Control catalogue, capability reasons and page token. |
| `GET /api/console/status` | Current dashboard observation snapshot; no motion authority. |
| `GET /api/console/planning` | Current draft map/mission state and job state. |
| `POST /api/console/action` | Revision-checked draft planning actions only. |

POSTs require the page token, JSON content type, a declared body no larger than
2 MB, and the current draft revision. Supported actions are `import_lap`,
`discard`, `fit`, `refit`, `move_handle`, `reset_handle`, `save_map`, `pose`,
`plan`, `save_mission`, and job `cancel`. Unknown launch, motor and calibration
actions fail; they are not passed to a shell or ROS.

## 4. How to preview locally

From the repository root:

```powershell
Set-Location src/risabot_automode
python -m risabot_automode.dashboard_panels.console_preview --port 8766
```

Then open `http://127.0.0.1:8766/console`.

This server binds only to loopback and shows no invented robot telemetry. The
planning tools are real; however, imported lap coordinates are operator input,
not proof of a calibrated venue frame.

When the code eventually runs inside the ROS dashboard, the preview route is
`http://<robot-ip>:8080/console` (or the configured dashboard port). Do not use
that on the car until the dashboard commit is reviewed, integrated and deployed
by the task that owns board lifecycle.

## 5. Tests and current result

Run the dashboard suite from the repository root:

```powershell
python -m pytest tests/test_dashboard*.py -q
node --check src/risabot_automode/risabot_automode/dashboard_panels/script/console.js
```

Coverage added includes:

- exact-stamp synchronization, partial/missing/stale images and bounded cache;
- source/view changes, obsolete-frame clearing and demand-driven conversion;
- aspect-ratio preservation and readable/scalable layout markers;
- mockup inventory parity and an explicit action or prerequisite for every
  screen button;
- malformed/oversized JSON, missing dependencies and unknown actions;
- stale browser revisions, conflicting edits and concurrent job protection;
- worker crash, timeout, cancellation and retry;
- map coverage, exact fingerprint, handle limits and map-change invalidation;
- off-road poses, body-preserving flip and incomplete mission refusal;
- successful use of the real supplied fit worker plus failed off-road planning;
- browser inspection of missing-lap and unavailable-launch messages.

The exact test count and pushed commit are recorded in the final task response;
rerun the command above after changing any dashboard behavior.

## 6. Known gaps — do not mark these complete in the UI

The following backend contracts do not exist in this V4 checkout yet:

1. A lifecycle supervisor that remains available while serially switching
   calibration/race stacks, prevents duplicate owners and reports observed
   readiness/failure.
2. A request/acknowledgement path to the real mode/motion owner for START,
   MANUAL, hand-back, STOP and E-STOP. `/auto_mode` is status, not a command API.
3. Versioned calibration jobs/sessions for sensor check, camera identity,
   intrinsics, extrinsics/IPM, LiDAR alignment, odometry/IMU, steering, PID,
   lighting, UWB, practice runs, keep-previous and rollback.
4. A validated map/mission/rules loader whose downstream consumers acknowledge
   the exact active revision. The current V4 mission does not consume these
   draft YAML files.
5. Mission-driven branch/corridor and V4 parking execution with identified
   started/completed/failed/cancelled results.
6. A verified venue-frame lap source with camera-to-map feedback disabled.
   `/v4_experimental/pose/local` is odometry-frame; coarse pose currently has a
   known orientation/header defect. Neither is suitable for venue lap recording.
7. Measured vehicle geometry. The supplied planner uses length 0.30 m, width
   0.192 m, wheelbase 0.216 m and rear overhang 0.042 m; these are not validated
   against this physical car.
8. Quick point-pair realignment and automatic per-control-point nudging. The
   supplied map builder explicitly leaves the latter unfinished.

Track these against `RISABOT5_AUTONOMY_TODO.md` items D02-D06 and E01-E05. The
control/integration task supplies the runtime adapters; the dashboard task then
wires them and adds inert acceptance tests before any physical trial.

## 7. Recommended continuation order

1. Re-run `tests/test_dashboard*.py` and open the loopback preview.
2. Review every disabled control's explanation against `console_contract.py`.
3. Finish visual parity for each calibration panel without adding fake values.
4. Agree request/response schemas with the control task. Include request ID,
   expected revision, accepted/rejected reason, observed state, job ID and loaded
   revision.
5. Wire one backend family at a time: lifecycle, authority, nonmoving
   calibration, moving calibration, map/mission activation, maneuver results.
6. For each family, test crash, timeout, cancellation, reconnect, stale response,
   conflict and restart behavior with inert transports.
7. Only after integration review, coordinate a board deployment and stationary
   validation. Hardware-moving calibration and autonomy remain supervised tests.

## 8. Safety and repository notes

- Do not publish `/auto_mode` to fake a mode transition.
- `/api/reset_odom` resets dashboard display counters only.
- `/api/record_playback` may request motion; it is not a harmless preview.
- A successful file write is not runtime activation.
- Do not use mockup example PASS values as calibration evidence.
- Do not commit unrelated dirty control, configuration, launch, media or
  calibration files with dashboard work.
- Do not deploy directly from a dirty checkout. Make a reviewed file manifest
  and coordinate with the owner of robot processes.

## 9. Related documents

- `DASHBOARD_V4_HANDOFF.md`: verified existing runtime contracts and ownership.
- `AUTONOMY_CONNECTION_AUDIT_2026-09-22.md`: end-to-end architecture gaps.
- `RISABOT5_AUTONOMY_TODO.md`: shared remaining work and acceptance tests.
- `dashboard_panels/CONSOLE_INTEGRATION.md`: implementation-level preview notes.
- `carbot_gui_mockup.html`: source visual/control mockup (reference input).
- `Setup Carbot.zip`: original planning-tool delivery (reference input).
