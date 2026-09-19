# RISA-bot V4 control — Stage 8

This package is the guarded motion boundary between V4 proposals and the
production actuator stack. It does not access motor hardware. It publishes
`/cmd_vel_v4_raw`; `cmd_safety_controller` remains the only path to
`/cmd_vel_auto`, and `servo_controller` retains manual override and its own
permit timeout.

## Competition behavior

- `LANE_FOLLOW`, `ROUNDABOUT`, and `HILL`: V4 trajectory steering.
- `LANE_RECOVERY`: V4 bounded recovery path.
- `PARALLEL_PARK` and `PERPENDICULAR_PARK`: V4 parking path when the mission
  state uses those states.
- `TUNNEL` and `OBSTRUCTION`: the existing specialized controller is passed
  through. This preserves autonomous completion while V4 remains the only lane
  tracker.
- traffic lights, boom gates, manual, finished, playback, and emergency states
  always request zero from Stage 8. Parking playback remains independently
  selected inside the production safety controller.

Every moving input is freshness checked. Invalid JSON, NaN/Inf values, stale
odometry, stale mission state, stale paths, source mismatches, e-stop, manual
mode, and unopened gates all publish zero.

## Safe startup

The package ships disabled and all Stage 8 gates ship false. Running the launch
file therefore cannot move the car:

```bash
ros2 launch risabot_v4_control v4_competition.launch.py
ros2 topic echo /v4_control/status
```

Keep `autonomy_source:=legacy` until calibration, replay, injected-proposal,
stop-preemption, timeout, and wheels-up checks pass. During an explicitly
supervised test, set `operator_motion_authorized: true` in
`config/v4_control.yaml`, rebuild, restart, and select V4:

```bash
ros2 launch risabot_v4_control v4_competition.launch.py autonomy_source:=v4
```

Setting the command source to V4 never falls back to a stale legacy command.
Loss of Stage 8 output stops the safety controller after `cmd_timeout`.
