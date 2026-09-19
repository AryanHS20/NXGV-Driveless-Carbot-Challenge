# V4 staged port

Each stage remains in shadow mode until its recorded-data checks pass. The
existing `risabot_automode` controller stays available throughout.

| Stage | Deliverable | Board output | Promotion evidence |
|---:|---|---|---|
| 0 | Sensor freshness monitor and isolated package | JSON diagnostics only | Build, topic audit, disconnected/fresh-input checks |
| 1 | Two-camera calibration and bird's-eye transforms (software complete; physical profiles pending) | Debug images and calibration report | Reprojection error and coverage measurements |
| 2 | Road mask, corridor and odometry-fixed recent memory implemented offline; physical threshold tuning pending | Mask, corridor, memory and confidence topics | Recorded track video with stale-frame tests |
| 3 | Local pose estimator plus duplicate-aware raw-range UWB bridge; UWB kept in a coarse global offset | Isolated UWB fix and local/global pose diagnostics | Survey, range/height calibration, frame alignment, recorded replay and physical point checks |
| 4 | Footprint-aware bicycle rollouts implemented; physical gates remain closed | JSON candidate summaries and rejection reasons | Synthetic geometry tests pass; replay and stationary wheels-up comparison still required |
| 5 | Reeds-Shepp parking proposals plus gated marking-based goal source | Proposed JSON parking path only | Physical marking thresholds, geometry, replay and repeated slow trials still required |
| 6 | Bounded reverse-and-rejoin recovery plus gated mission-policy request source | Proposed recovery path only | Rules review, rear coverage and stopped/attempt policy validation |
| 7 | Diagnostic command arbitration implemented; all promotion gates closed | JSON proposal only | Full safety review and explicit physical validation before any motion interface |
| 8 | Guarded executor in separate `risabot_v4_control` package plus explicit safety-controller source selection | `/cmd_vel_v4_raw`, still behind the production safety envelope | Injected-proposal tests, stop/timeout fault injection, wheels-up validation, then supervised low-speed trials |

The simulator assumes three cameras. The physical adaptation begins with the two
available MIPI cameras; blind regions must be measured before later stages.
