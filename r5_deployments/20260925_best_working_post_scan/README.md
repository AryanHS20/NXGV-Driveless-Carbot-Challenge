# R5 best working code — post-`/scan`-veto restore

**Current preferred R5 board baseline, 25 September 2026.** These 13 source files are byte-for-byte copies from Risabot 5 after the P204 restore. They supersede the `20260925_parallel_hill` snapshot for the active car. `SHA256SUMS` records their exact hashes.

This is a board-matched reference snapshot. The main `src/` checkout also contains R1 and other work; do not bulk-copy it onto R5. The full board backup is preserved outside Git at `/home/sunrise/risabotcar_ws/track_test_backups/pre_parking_recorded_20260925/full_state.tar.gz` (SHA-256 `8f6af40c88045e6b5702ebd54d87414395f9be12143029a05726846acdcb94d2`). The complete immediately-pre-restore backup is `/home/sunrise/risabotcar_ws/track_test_backups/pre_restore_post_scan_20260925_1732/full_state.tar.gz` (SHA-256 `e4603a6c94c4f348c96f34351168314995cf5ec0d540dc4030b7b2efc151a6dd`). These large backups are also in the local Competition workspace, not Git.

| Snapshot file | Board path under `/home/sunrise/risabotcar_ws/src/` |
| --- | --- |
| `servo_controller.py` | `control_servo/control_servo/servo_controller.py` |
| `cmd_safety_controller.py` | `risabot_automode/risabot_automode/cmd_safety_controller.py` |
| `setup.py` | `risabot_automode/setup.py` |
| `signage_detector.py` | `risabot_automode/risabot_automode/signage_detector.py` |
| `auto_driver.py` | `risabot_automode/risabot_automode/auto_driver.py` |
| `track_test_config.py` | `risabot_v4_control/risabot_v4_control/track_test_config.py` |
| `track_test.launch.py` | `risabot_v4_control/launch/track_test.launch.py` |
| `motion_executor.py` | `risabot_v4_control/risabot_v4_control/motion_executor.py` |
| `hill_boost_core.py` | `risabot_v4_control/risabot_v4_control/hill_boost_core.py` |
| `speed_core.py` | `risabot_v4_control/risabot_v4_control/speed_core.py` |
| `road_mask_core.py` | `risabot_v4_experimental/risabot_v4_experimental/road_mask_core.py` |
| `road_mask_shadow.py` | `risabot_v4_experimental/risabot_v4_experimental/road_mask_shadow.py` |
| `trajectory_shadow.py` | `risabot_v4_experimental/risabot_v4_experimental/trajectory_shadow.py` |

The active R5 configuration disables the LiDAR obstacle collision veto (`use_lidar_obstacles=false`) while retaining the raw scan for display/diagnostics. `road_mask_core.py` retains the P160 green-mat exclusion. Normal AUTO motor duty is 50% straight / 44% turn. The P160 sign-timed hill pulse remains. The launch retains the P202 `publish_boom_state=True` gate fix. The P167/P198 named parking playback and automatic parallel-sign trigger were removed from the active board code; the user's recorded maneuvers are preserved separately on the car.

The guarded P204 build of `control_servo`, `risabot_automode` and `risabot_v4_control` passed. After restart, V4 was active with MANUAL, zero command, fresh OPEN gate, and 50%/44% duty. The green-mask code passed synthetic and saved-frame checks. **An AUTO drive after this restore and a physical green-mat pass have not been established.** “Best working” names the user's chosen current code baseline, not a newly verified full-course pass.

The service owns the hardware connection. On R5, start/check the existing stack with:

```bash
sudo systemctl start risabot5-track-stack.service
systemctl is-active risabot5-track-stack.service
```

The operator selects AUTO for attended driving. Do not start a second direct ROS launch alongside the service.
