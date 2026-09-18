# Stage 5 measured parking-goal contract

Stage 5 accepts a measured target on `/v4_experimental/parking/goal` as a
`std_msgs/String` containing JSON. The experimental `parking_goal_source`
measures this contract from closed bright markings in the calibrated secondary
BEV image. It is disabled and its threshold, geometry, and coverage gates must
be validated before it can publish a goal.

```json
{
  "kind": "parallel",
  "frame_id": "base_link",
  "x_m": 0.42,
  "y_m": -0.25,
  "yaw_rad": 1.5708,
  "slot_length_m": 0.69,
  "slot_width_m": 0.47,
  "confidence": 0.92,
  "image_stamp_sec": 1234.56
}
```

- `x_m`, `y_m`, and `yaw_rad` describe the desired **rear-axle pose** in the
  vehicle base frame at `image_stamp_sec`; positive `x` is forward and positive
  `y` is left.
- `kind` is `parallel` or `perpendicular`.
- Slot dimensions must be measured clear interior dimensions.
- `image_stamp_sec` must match the secondary fused road-mask frame within the
  configured tolerance.
- The complete vehicle footprint plus clearance must fit inside the slot.

The output `/v4_experimental/parking/proposed_path` is JSON for visualization
and replay only. Each point contains position, heading, direction, and
curvature. It cannot be executed by this package.
