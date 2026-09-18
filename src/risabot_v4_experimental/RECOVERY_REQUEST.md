# Stage 6 recovery request contract

Stage 6 accepts a JSON diagnostic request on
`/v4_experimental/recovery/request`. No current mission node publishes this
contract and the corresponding validation gates ship closed.
The node also requires a fresh Stage 4 report containing at least one evaluated
candidate and proving that all forward candidates were rejected.

```json
{
  "problem": "no_forward_candidate",
  "hard_hold": "",
  "permitted": true,
  "stopped": true,
  "attempts": 0,
  "image_stamp_sec": 123.45,
  "frame_id": "base_link"
}
```

- `problem` names the condition that exhausted forward-only planning.
- Any non-empty `hard_hold` blocks recovery. Emergency stop, stale camera,
  unresolved traffic light, manual stop, and health faults must be represented
  as hard holds by the future validated request source.
- `permitted` must come from reviewed mission logic; `stopped` must confirm the
  vehicle is stationary before planning.
- `attempts` prevents repeated recovery loops. The checked-in limit is three.
- `image_stamp_sec` must match the secondary road and rear-coverage frame.
- `frame_id` is fixed to the current `base_link` frame.

The node publishes JSON diagnostics and proposed geometry only. It cannot
execute a path or publish a vehicle command.
