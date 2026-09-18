# V4 staged port

Each stage remains in shadow mode until its recorded-data checks pass. The
existing `risabot_automode` controller stays available throughout.

| Stage | Deliverable | Board output | Promotion evidence |
|---:|---|---|---|
| 0 | Sensor freshness monitor and isolated package | JSON diagnostics only | Build, topic audit, disconnected/fresh-input checks |
| 1 | Two-camera calibration and bird's-eye transforms (software complete; physical profiles pending) | Debug images and calibration report | Reprojection error and coverage measurements |
| 2 | Road mask, corridor and odometry-fixed recent memory implemented offline; physical threshold tuning pending | Mask, corridor, memory and confidence topics | Recorded track video with stale-frame tests |
| 3 | Local pose estimator implemented; UWB kept in a coarse global offset | Local/global pose diagnostics | Core noise/outlier tests pass; recorded replay still required |
| 4 | Footprint-aware bicycle rollouts implemented; physical gates remain closed | JSON candidate summaries and rejection reasons | Synthetic geometry tests pass; replay and stationary wheels-up comparison still required |
| 5 | Reeds-Shepp parking planner | Proposed parking path only | Measured geometry and repeated slow parking trials |
| 6 | Bounded reverse-and-rejoin recovery | Proposed recovery path only | Rules review, rear coverage, obstacle veto tests |
| 7 | Reviewed command arbitration integration | Gated motion request | Full safety review and explicit physical validation |

The simulator assumes three cameras. The physical adaptation begins with the two
available MIPI cameras; blind regions must be measured before later stages.
