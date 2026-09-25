# R5 board snapshot: parallel sign and hill pulse (25 September 2026)

**Historical snapshot:** The active R5 board was restored to the [post-`/scan`-veto baseline](../20260925_best_working_post_scan/README.md) at P204. This parking-trigger snapshot remains for history and is not the current board code.

These files are byte-for-byte copies of the R5 source verified after the
automatic parallel-sign fix. The main `src/` checkout also contains R1 work;
these files are kept separately so a Git checkout does not overwrite either
vehicle's current configuration. This directory is a reference snapshot, not
an instruction to bulk-copy source to a board.

| File | Board destination under `/home/sunrise/risabotcar_ws/src/` |
| --- | --- |
| `parallel_park_trigger.py` | `risabot_automode/risabot_automode/parallel_park_trigger.py` |
| `signage_detector.py` | `risabot_automode/risabot_automode/signage_detector.py` |
| `setup.py` | `risabot_automode/setup.py` |
| `servo_controller.py` | `control_servo/control_servo/servo_controller.py` |
| `track_test_config.py` | `risabot_v4_control/risabot_v4_control/track_test_config.py` |
| `track_test.launch.py` | `risabot_v4_control/launch/track_test.launch.py` |
| `motion_executor.py` | `risabot_v4_control/risabot_v4_control/motion_executor.py` |
| `hill_boost_core.py` | `risabot_v4_control/risabot_v4_control/hill_boost_core.py` |

`parking_movement_20260925_1602.json` is the exact named parallel recording:
919 motor PWM and steering samples over 45.95 seconds; SHA-256
`111adddab48fa029907adce7573cfdc8a9f3c807af4b095448ac07fd9319f90d`.
The trigger requests only this recording after two continuous seconds of a
parallel sign in operator-selected AUTO, with the existing permit, gate and
playback checks. Hill speed is doubled for two seconds after two seconds of
hill-sign visibility. These are software and stationary runtime results;
neither parking fit nor hill driving has a physical pass.

The deployed full V4 stack is managed by `risabot5-track-stack.service`. On R5:

```bash
sudo systemctl start risabot5-track-stack.service
systemctl is-active risabot5-track-stack.service
```

The service owns the hardware connection. Do not run a second direct ROS launch
alongside it. The operator selects AUTO on the car.

After the first restart, the R5 launch setting of `publish_boom_state: False`
left the gate state stale and held AUTO at zero. The deployed launch file in
this snapshot now sets it to `True` (SHA-256
`f3dbd593b75e6c8848bac8d76838a7e2c257d9b9dcf510ab995deede658cdb89`).
After a verification restart, the service was active and the dashboard showed
fresh OPEN gate readings over more than three road frames in MANUAL with zero
motor command. Physical AUTO motion was not tested by this change.
