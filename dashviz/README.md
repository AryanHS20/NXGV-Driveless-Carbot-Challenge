# RISA Drive Viz (React)

Tesla-style driving visualization for the RISA-bot dashboard.
**Display only** — reads `/data` + `/lidar_data`, commands nothing.

## Develop

```bash
cd dashviz
npm install
npm run dev      # local dev server (proxies /data to the robot)
npm test         # render smoke test (mock telemetry, no robot)
npm run build    # self-contained dist/index.html (double-clickable)
```

Without a robot nearby (or from `file://`), the app runs on built-in
mock telemetry and labels itself MOCK. Served from the dashboard it
switches to live data automatically.

## Deploy to the board

Commit `dist/index.html` (already bundled, no `npm` needed on the
robot). Serve the directory from the dashboard static routes (planned
`/viz/` mount, same pattern as `/sim/`).

## Data contract

- `GET /data` — speed, speed_pct, auto_mode, state, lane_error,
  lane_lost, traffic_light, stale_streams, health_stale, odom_x/y/yaw
- `GET /lidar_data` — `{points: [[x_fwd_m, y_left_m], …]}`
- Unknown fields are ignored; missing fields render as unavailable.
