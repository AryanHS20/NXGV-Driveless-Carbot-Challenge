# Simulator replay format v1 (read-only, display only)

This is the contract between the robot side (dashboard + exporter, owned here)
and the simulator side (loader/follow mode, owned by the sim team). Nothing in
this document grants motion authority; replays are observed data for review.

## Endpoints (robot dashboard, port 8080)

- `GET /api/replay/list` → `{"runs": ["run_….jsonl", …], "replays": ["….replay.json", …]}` (newest first, capped at 50)
- `GET /api/replay/get?name=<file>` → the replay JSON below (name must be a
  plain `.json` filename inside `~/risabot_maps`; anything else is rejected)
- `GET /sim/index.html` → the bundled simulator UI (static files only)

## Replay JSON schema (version 1)

```json
{
  "version": 1,
  "source_run": "run_20260918_082444.jsonl",
  "frames": [
    {
      "t": 1789719885.01,
      "odom": {"x": 0.12, "y": 0.01, "yaw": 0.05},
      "speed": 0.15,
      "lane_error": -0.04,
      "curvature": 0.01,
      "lane_lost": false,
      "tl": "green",
      "hill": false,
      "tunnel": false,
      "corridor": [
        {"forward_m": 0.5, "left_m": 0.02, "width_m": 0.30}
      ]
    }
  ]
}
```

Rules: `odom` is `null` when no odometry was recorded for that frame.
`corridor` is `[]` when no V4 status snapshot matched (forward camera,
near-to-far samples). Frames are chronological at ≤ the recorder rate
(5 Hz); long runs are stride-downsampled to ≤ 20000 frames by the exporter.

## Sim loader contract (to implement on the sim side)

- Fetch `/api/replay/get?name=<file>` (same origin when iframed, so no CORS issue).
- Validate `version === 1`, then step frames by `t` at chosen playback speed.
- Map `odom` → actual-pose marker, `corridor` → corridor overlay,
  `lane_error`/`tl`/flags → event timeline. Unknown fields must be ignored
  so v2 can extend without breaking v1 loaders.
- Suggested URL hook: `/sim/index.html?replay=<file>` auto-loads that replay.

## Exporter

```bash
python3 tools/export_replay.py ~/risabot_maps/run_<UTC>.jsonl \
  [--status-log status.jsonl] [--out replay.json] [--max-frames 20000]
```

The optional status sidecar is one JSON object per line:
`{"t_wall": <float>, "status": {<road/status payload>}}`. Snapshots merge
into frames by nearest timestamp within 0.5 s; unmatched frames keep
`"corridor": []`. To capture one later: subscribe
`/v4_experimental/road/status` and append `{"t_wall": time.time(),
"status": <parsed payload>}` per message.
