# Carbot Simulator — V4

Extract the ZIP to a new folder, then double-click `index.html`.
No install, administrator access, terminal, ROS, server or internet is required.
`START_HERE.html` contains button-by-button instructions.

Click **20 cm**, then **Run both missions**. Choose the map and local-planner
views. UWB now affects a separate coarse global estimate; camera boundaries
and odometry guide local motion. Cyan shows observed corridor centres; grey
shows evaluated candidates; blue shows the selected trajectory. Yellow is the
local navigation pose, violet the coarse global estimate, green the actual pose.

The car remains 30 × 19.2 cm with a 21.6 cm wheelbase and an adjustable default
40 cm rear-axle turning radius. All photo-based lane-change and bay dimensions
are retained. Exactly one command owner applies safety and the 200 ms watchdog.

This model uses a known start and a prior map. It is not map-free navigation,
a complete EKF/SLAM implementation, or a calibrated real-vehicle dynamics model.
See `VALIDATION.md` for actual test results and remaining limitations.

Developers: `npm ci`, `node build.mjs`, and `npm test`.
Runtime: bundled Three.js 0.186.0 / Rapier 0.17.3; licences included.

V4 adds a live 0–5 cm paint-boundary allowance, strict-first candidate selection,
and automatic brake → short reverse → stopped gear change → forward rejoin
recovery. Both features start enabled. Physical obstacles and sensor/emergency
holds stay hard limits. Original-boundary crossings are counted separately from
physical contacts. These are training overrides, not rulebook amendments.

Read the new V4 section in START_HERE.html and current VALIDATION.md.
