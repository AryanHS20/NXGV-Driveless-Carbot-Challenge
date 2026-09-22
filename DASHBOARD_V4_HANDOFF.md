# Dashboard handoff for the active V4 checkout

2026-09-22. Target: `NXGV-Driveless-Carbot-Challenge`, `feat/safety-contract-rework`, current working tree. The user's request is a functional remodel of the supplied mockup, including workflows. Preserve the separate V2 work as reference; do not silently migrate the car.

## Ownership

- Dashboard task: `src/risabot_automode/risabot_automode/dashboard.py`, `dashboard_panels/**`, `tests/test_dashboard*.py`, dashboard goldens, UI/session/planning interfaces.
- Control/integration task: mission, trajectory, actuator/mode ownership, pose, calibration runtime adapters, configuration, launches, deployment and physical testing.
- Calibration evidence remains car-specific. R5 front profile is `risabot5_camera_profiles.yaml`, native320, SHA-256 `fe599255c1ebdb6c03e50f216f534db1a7e29e7795bad179d38d4fe771326a99`; trim80. The board active file is `/home/sunrise/risabot5_profiles/camera_profiles.yaml`.
- Work locally with inert transports. Coordinate any board rebuild, camera restart, launch lifecycle or actuator operation through the control task.

## Existing interfaces verified in source

| Interface | Current semantics |
| --- | --- |
| `GET /data` | Dashboard's current observed data and freshness; extend actual subscriptions when needed. It is not automatically a complete 16-block telemetry API. |
| `GET /camera_feed`, `/api/set_cam_view` | Live camera/processed view selection, including synchronized V4 image tiles. A display switch is not a calibration or camera identity assignment. |
| `GET /api/v4_telemetry` | Local/coarse poses, road/trajectory/scan-derived diagnostic payload. Frames must stay explicit. |
| `GET /api/get_param`, `POST /api/set_param` | ROS parameter transport; honor response and read-back. Several control/test settings require restart and reject live edits. |
| `POST /api/save_defaults` | Existing defaults persistence; not a versioned calibration-session activation protocol. |
| `POST /api/reset_competition` | Sends RESET/LAP1/LAP2 to `/set_challenge`; response means sent, not armed or completed. |
| `POST /api/reset_odom` | **Display counters only.** Does not reset hardware localization/map alignment. |
| `POST /api/calibrate_imu` | Sends JSON to `/imu/calibrate`; inspect `/imu/rpy` measurements and parameter read-back. There is no correlated calibration-job completion acknowledgment here. Sending is not PASS. |
| `POST /api/record_playback` | Existing teaching/recording/playback mechanism; playback can request motion. It is not computed V4 parking or an innocuous preview operation. |
| `/auto_mode` | Servo owner's authoritative heartbeat. Publishing a fake value here does not change the servo's internal mode. A mode request/ack adapter is required for UI control. |
| `/e_stop`, `/motion_permitted`, `/cmd_safety_status`, `/dashboard_state` | Existing control signals/status. Observe the owner/result; never synthesize a mission state to make a panel green. |
| `/v4_control/status` and `/v4_experimental/{bev,road,pose,trajectory,arbitration}/status` | Real stage state/reasons. Additional goal/parking/recovery statuses exist only when those stages run. |

## Units and localization contract

Front-road BEV and local maneuver paths use rear-axle ground coordinates: metres, +x forward, +y left, yaw left-positive. Motor command `angular.z` uses normalized RIGHT-positive steering and is not yaw rate. Test requested motor duty is 65%; it is not 0.65 m/s. Manual gear/duty is a separate control.

`/v4_experimental/pose/local` is an odometry-frame estimate; `/v4_experimental/pose/coarse` is a coarse UWB-aligned frame, not automatically a validated venue/map frame. Current V4 pose mirrors `/odom`; it does not automatically use the separate `/odom_fused`. The audit reproduces a coarse-orientation/header defect, assigned to the control task. **Do not export either pose as a calibrated venue lap until frame alignment and that defect are resolved.** Diagnostic traces may still be shown with honest frame labels and readiness.

## Missing runtime adapters: explicit backend work

These contracts are requirements, not implemented APIs. Keep their UI and inert tests prepared while the control task implements them.

1. **Lifecycle supervisor:** an independently serving supervisor manages its own camera/control children, reports starting/manual/ready/stopping/failed, serializes switches and exposes bounded logs/cancel/retry. Existing `track_test.launch.py` is a lane test, not the complete race launch or calibration wizard.
2. **Mode/motion action adapter:** requests reach the actual servo/control owner; replies include accepted/rejected, reason, request ID and observed state. START, MANUAL, STOP and E-STOP require defined semantics. Reconnect never replays actions; the owner controls whether a transition is valid.
3. **Calibration jobs:** named vehicle/sensor session; capture/compute/review/save/cancel; measured result/error/evidence; dependency invalidation on mount/resolution/geometry changes; atomic activation only after consumers confirm the loaded revision. Hardware-moving jobs require their explicit operator-run workflow, not mock progress timers.
4. **Map/mission/rules activation:** schema/units/frames validation; exact map fingerprint; required mission rules; consumer load acknowledgment with revision. Saving YAML is only file export until this exists. The active ROS mission currently does not consume the new format.
5. **Maneuver result/status:** goal/plan identity, route/checkpoint, started/completed/failed/cancelled, actual active command source and reason. Needed to distinguish computed parking from recorded playback and to count actual recovery attempts.

Suggested request fields for these new adapters: `request_id`, action, vehicle/session ID, expected active revision and action-specific inputs. Suggested response fields: accepted, observed state, reason, job ID and loaded revision. Exact transport/API naming will be agreed at implementation; do not assume V2 `/carbot/*` services exist here.

## Work the dashboard task can do now

Build the requested layout and all navigation; retain the mockup control inventory. Wire current observed telemetry, camera views, parameter get/set acknowledgements and existing documented actions. Implement local session/file/planning views, map/mission editing and export using the supplied helpers, with export distinct from runtime activation. Add backend-derived prerequisites and explicit integration-pending explanations for the missing adapters; do not present these as completed workflows.

Test with inert transport: every control/action, failed/malformed/stale responses, reconnect/reload, camera disconnection, parameter rejection, job cancellation/timeouts, concurrent save, map revision changes, split view/mobile layout and hidden-tab throttling. Preserve evidence provenance and distinguish requested values from measurements. Run real browser tests against the local dashboard before integration.

Acceptance: a visible control either performs its defined backend action with observed result or precisely states the remaining prerequisite/adapter. Missing adapters remain open TODOs, so this intermediate UI must not be called the completed functional remodel.

Read `AUTONOMY_CONNECTION_AUDIT_2026-09-22.md` for confirmed runtime gaps and `RISABOT5_AUTONOMY_TODO.md` E01-E05 for completion tests.
