import { useEffect, useRef, useState } from 'react';

const EMPTY = {
  speed: 0, speed_pct: null, auto_mode: false, state: '',
  lane_error: 0, lane_lost: false, traffic_light: 'unknown',
  stale_streams: [], health_stale: [],
  odom_x: 0, odom_y: 0, odom_yaw: 0,
};

let mockT = 0;
function mockFrame() {
  mockT += 0.5;
  const lidar = [];
  for (let i = 0; i < 36; i++) {
    const a = (i / 36) * Math.PI * 2;
    const r = 1.5 + (i % 4) * 0.6;
    lidar.push([Math.cos(a) * r + 2.4, Math.sin(a) * r]);
  }
  lidar.push([0.38, -0.35]);
  lidar.push([2.2, 0.9]);
  return {
    data: {
      ...EMPTY,
      speed: Math.max(0, 0.16 + 0.05 * Math.sin(mockT * 0.5)),
      speed_pct: 25,
      auto_mode: true,
      state: 'LANE_FOLLOW',
      lane_error: Math.sin(mockT * 0.7) * 0.3,
      traffic_light: ['green', 'green', 'yellow', 'red'][Math.floor(mockT / 8) % 4],
      stale_streams: Math.sin(mockT * 0.23) > 0.75 ? ['camera_frame'] : [],
      odom_x: mockT * 0.08,
      odom_yaw: Math.sin(mockT * 0.2) * 0.2,
    },
    lidar,
    live: false,
  };
}

async function getJSON(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error('http ' + res.status);
  return res.json();
}

/** Poll robot endpoints; fall back to mock telemetry when unreachable. */
export function useTelemetry(pollMs = 500) {
  const [snap, setSnap] = useState(() => ({...mockFrame(), live: false}));
  const timer = useRef(0);
  useEffect(() => {
    let dead = false;
    async function tick() {
      try {
        const [data, lidarDoc] = await Promise.all([
          getJSON('/data'),
          getJSON('/lidar_data'),
        ]);
        if (!dead) setSnap({data: {...EMPTY, ...data}, lidar: lidarDoc.points || [], live: true});
      } catch {
        if (!dead) setSnap(mockFrame());
      }
    }
    tick();
    timer.current = setInterval(tick, pollMs);
    return () => { dead = true; clearInterval(timer.current); };
  }, [pollMs]);
  return snap;
}

/** Normalize a lidar point (array or {x,y}) to {x, y} metres. */
export function normPt(p) {
  if (Array.isArray(p)) return {x: +p[0] || 0, y: +p[1] || 0};
  return {x: +p.x || 0, y: +p.y || 0};
}

/** Cluster points into 0.3 m grid cells, capped, nearest-cluster red flag. */
export function clusterObstacles(points, cell = 0.3, cap = 14) {
  const cells = new Map();
  for (const raw of points) {
    const p = normPt(raw);
    if (p.x < -0.6 || p.x > 7 || Math.abs(p.y) > 4.2) continue;
    const key = Math.round(p.x / cell) + ':' + Math.round(p.y / cell);
    const hit = cells.get(key) || {x: 0, y: 0, n: 0};
    hit.x += p.x; hit.y += p.y; hit.n++;
    cells.set(key, hit);
  }
  return [...cells.values()]
    .map(c => ({x: c.x / c.n, y: c.y / c.n, n: c.n,
                d: Math.hypot(c.x / c.n, c.y / c.n)}))
    .sort((a, b) => a.d - b.d)
    .slice(0, cap);
}
