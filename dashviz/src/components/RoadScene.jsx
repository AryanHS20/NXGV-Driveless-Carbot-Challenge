import React, { useRef, useEffect, useCallback } from 'react';
import { clusterObstacles } from '../api.js';

const W = 400, H = 560, CX = W / 2, HORIZON = 140, ROAD_BOT = H - 56;

/**
 * Project ground coordinates (x forward m, y left m) into scene pixels.
 * latShift moves the whole scene laterally to visualise lane error.
 */
function project(x, y, latShift) {
  const t = Math.max(0, Math.min(1, 1 - x / 7));
  return {
    sx: CX + (y / 4.2) * 150 - (latShift || 0) * (1 - t * 0.75),
    sy: ROAD_BOT - t * (ROAD_BOT - HORIZON),
    t,
  };
}

function lanePath(side, latShift) {
  const pts = [];
  for (let i = 0; i <= 24; i++) {
    const x = 0.2 + (i / 24) * 6.5;
    pts.push(project(x, side * (0.55 + x * 0.045), latShift));
  }
  return pts;
}

/* ── Canvas drawing helpers ──────────────────────────────────────────── */

function drawBackdrop(ctx) {
  // Sky gradient — light Tesla white
  const sky = ctx.createLinearGradient(0, 0, 0, HORIZON + 20);
  sky.addColorStop(0, '#f8f9fb');
  sky.addColorStop(1, '#e8ecf2');
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, W, HORIZON + 20);

  // Road surface gradient
  const road = ctx.createLinearGradient(0, HORIZON, 0, H);
  road.addColorStop(0, '#dde3ec');
  road.addColorStop(0.3, '#d4dbe6');
  road.addColorStop(1, '#c8d0dc');
  ctx.fillStyle = road;

  // Perspective trapezoid
  const roadTopHalf = 80;
  ctx.beginPath();
  ctx.moveTo(CX - roadTopHalf, HORIZON);
  ctx.lineTo(0, H);
  ctx.lineTo(W, H);
  ctx.lineTo(CX + roadTopHalf, HORIZON);
  ctx.closePath();
  ctx.fill();

  // Horizon line
  ctx.strokeStyle = 'rgba(180, 190, 210, 0.5)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, HORIZON);
  ctx.lineTo(W, HORIZON);
  ctx.stroke();
}

function drawGridLines(ctx, dashOffset, speed) {
  ctx.save();
  ctx.strokeStyle = 'rgba(160, 172, 195, 0.18)';
  ctx.lineWidth = 1;
  for (let i = 0; i < 8; i++) {
    const raw = ((i / 8) + (dashOffset % 50) / 50 / 8) % 1;
    const t = raw * raw; // quadratic for perspective spacing
    const y = ROAD_BOT - t * (ROAD_BOT - HORIZON);
    // Perspective narrowing
    const spread = 1 - t * 0.6;
    const x1 = CX - 200 * spread;
    const x2 = CX + 200 * spread;
    ctx.globalAlpha = 0.15 + raw * 0.6;
    ctx.beginPath();
    ctx.moveTo(x1, y);
    ctx.lineTo(x2, y);
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
  ctx.restore();
}

function strokeLane(ctx, points, color, width, glow) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  if (glow) {
    ctx.shadowColor = color;
    ctx.shadowBlur = 16;
  }
  ctx.beginPath();
  points.forEach((p, i) => {
    i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
  });
  ctx.stroke();
  ctx.restore();
}

function drawCorridor(ctx, left, right, laneCol, bad) {
  // Ribbon fill
  ctx.save();
  ctx.globalAlpha = bad ? 0.12 : 0.10;
  ctx.fillStyle = laneCol;
  ctx.beginPath();
  left.forEach((p, i) => {
    i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
  });
  for (let i = right.length - 1; i >= 0; i--) {
    ctx.lineTo(right[i].sx, right[i].sy);
  }
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // Lane boundaries
  strokeLane(ctx, left, laneCol, 4.5, true);
  strokeLane(ctx, right, laneCol, 4.5, true);
}

function drawYellowEdges(ctx, latShift) {
  [-1.9, 1.9].forEach(side => {
    const pts = lanePath(side, latShift);
    ctx.save();
    ctx.strokeStyle = '#d4a60a';
    ctx.lineWidth = 2.2;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.globalAlpha = 0.45;
    ctx.beginPath();
    pts.forEach((p, i) => {
      i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
    });
    ctx.stroke();
    ctx.restore();
  });
}

function drawCenterGuide(ctx, latShift, errShown, dashOffset) {
  const pts = [];
  for (let i = 0; i <= 24; i++) {
    const x = 0.2 + (i / 24) * 6.5;
    pts.push(project(x, errShown * 0.4, latShift));
  }
  ctx.save();
  ctx.strokeStyle = 'rgba(140, 155, 185, 0.35)';
  ctx.lineWidth = 1.8;
  ctx.lineCap = 'round';
  ctx.setLineDash([22, 28]);
  ctx.lineDashOffset = -dashOffset;
  ctx.beginPath();
  pts.forEach((p, i) => {
    i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
  });
  ctx.stroke();
  ctx.restore();
}

function drawTrafficSignals(ctx, latShift, activeLamp) {
  const lampCol = { red: '#e5484d', yellow: '#f5b301', green: '#46a758' }[activeLamp];
  [-1, 1].forEach(side => {
    const s = project(5.5, side * 2.2, latShift);
    // Pole
    ctx.save();
    ctx.strokeStyle = '#8b95a5';
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(s.sx, s.sy);
    ctx.lineTo(s.sx, s.sy - 50);
    ctx.stroke();
    // Housing
    ctx.fillStyle = '#2a3342';
    ctx.beginPath();
    roundRect(ctx, s.sx - 8, s.sy - 76, 16, 28, 5);
    ctx.fill();
    // Lamps
    ['red', 'yellow', 'green'].forEach((name, i) => {
      const active = activeLamp === name;
      ctx.fillStyle = active ? lampCol : '#4a5465';
      if (active) {
        ctx.shadowColor = lampCol;
        ctx.shadowBlur = 14;
      } else {
        ctx.shadowColor = 'transparent';
        ctx.shadowBlur = 0;
      }
      ctx.beginPath();
      ctx.arc(s.sx, s.sy - 68 + i * 9, 3.2, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();
  });
}

function drawObstacles(ctx, obstacles, latShift) {
  obstacles.forEach(o => {
    const s = project(o.x, o.y, latShift);
    const size = 10 + (1 - Math.min(1, o.d / 6)) * 26;
    const fill = o.d < 0.5 ? '#e5484d' : o.d < 1.1 ? '#f5b301' : '#b0b9c8';

    ctx.save();
    // Shadow
    ctx.fillStyle = 'rgba(20,30,50,0.12)';
    ctx.beginPath();
    roundRect(ctx, s.sx - size / 2 + 2, s.sy - size * 1.1 + 3, size, size * 1.15, size * 0.2);
    ctx.fill();
    // Body
    ctx.fillStyle = fill;
    if (o.d < 1.1) {
      ctx.shadowColor = fill;
      ctx.shadowBlur = 12;
    }
    ctx.globalAlpha = o.n > 2 ? 1 : 0.55;
    ctx.beginPath();
    roundRect(ctx, s.sx - size / 2, s.sy - size * 1.15, size, size * 1.15, size * 0.22);
    ctx.fill();
    // Windshield
    ctx.shadowBlur = 0;
    ctx.fillStyle = 'rgba(20,28,40,0.45)';
    ctx.beginPath();
    roundRect(ctx, s.sx - size * 0.32, s.sy - size * 0.95, size * 0.64, size * 0.38, size * 0.12);
    ctx.fill();
    ctx.restore();
  });
}

function drawEgoCar(ctx) {
  const w = 64, h = 108;
  const x = CX - w / 2, y = ROAD_BOT - h + 16;

  ctx.save();
  // Drop shadow
  ctx.fillStyle = 'rgba(20,30,50,0.15)';
  ctx.beginPath();
  roundRect(ctx, x + 4, y + 6, w, h, 16);
  ctx.fill();

  // Body
  const body = ctx.createLinearGradient(x, y, x + w, y);
  body.addColorStop(0, '#9aa7ba');
  body.addColorStop(0.45, '#eef2f7');
  body.addColorStop(1, '#8b98ac');
  ctx.fillStyle = body;
  ctx.beginPath();
  roundRect(ctx, x, y, w, h, 15);
  ctx.fill();

  // Body highlight edge
  ctx.strokeStyle = 'rgba(255,255,255,0.65)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  roundRect(ctx, x + 1, y + 1, w - 2, h - 2, 14);
  ctx.stroke();

  // Glasshouse
  const glass = ctx.createLinearGradient(0, y + 20, 0, y + 58);
  glass.addColorStop(0, '#0d1626');
  glass.addColorStop(1, '#1d2f47');
  ctx.fillStyle = glass;
  ctx.beginPath();
  roundRect(ctx, x + 10, y + 20, w - 20, 38, 10);
  ctx.fill();

  // Windshield reflection
  ctx.fillStyle = 'rgba(140,200,255,0.28)';
  ctx.beginPath();
  roundRect(ctx, x + 14, y + 24, w - 28, 12, 6);
  ctx.fill();

  // Wheels
  ctx.fillStyle = '#141a24';
  [[-7, 12], [w - 5, 12], [-7, h - 20], [w - 5, h - 20]].forEach(([dx, dy]) => {
    ctx.beginPath();
    roundRect(ctx, x + dx, y + dy, 12, 16, 4);
    ctx.fill();
  });

  // Headlight bar
  ctx.fillStyle = 'rgba(190,225,255,0.85)';
  ctx.shadowColor = '#bfe1ff';
  ctx.shadowBlur = 10;
  ctx.beginPath();
  roundRect(ctx, x + 12, y + h - 7, w - 24, 4, 2);
  ctx.fill();

  ctx.restore();
}

/** Polyfill-safe roundRect that works on all browsers. */
function roundRect(ctx, x, y, w, h, r) {
  r = Math.min(typeof r === 'number' ? r : 4, w / 2, h / 2);
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

/* ── Main Component ──────────────────────────────────────────────────── */

/**
 * Tesla-white animated canvas driving scene.
 * Display only — reads data + lidar, never commands.
 */
export default function RoadScene({ data, lidar, speed }) {
  const canvasRef = useRef(null);
  const stateRef = useRef({
    errShown: 0,
    dashOffset: 0,
    lastTime: 0,
  });

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return; // jsdom has no canvas implementation
    const s = stateRef.current;
    const now = performance.now();
    const dt = s.lastTime ? (now - s.lastTime) / 1000 : 0.016;
    s.lastTime = now;

    const err = Math.max(-1, Math.min(1, +data.lane_error || 0));
    s.errShown += (err - s.errShown) * Math.min(1, dt * 8); // smooth lerp
    s.dashOffset += (speed || 0) * dt * 180; // scroll with speed

    const latShift = s.errShown * 52;
    const stale = (data.stale_streams || []).length > 0;
    const bad = stale || data.lane_lost === true;
    const laneCol = bad ? '#e5484d' : '#2f7cf6';
    const lamp = String(data.traffic_light || 'unknown');

    const left = lanePath(-1, latShift);
    const right = lanePath(1, latShift);
    const obstacles = clusterObstacles(lidar || []);

    // Clear + draw
    ctx.clearRect(0, 0, W, H);
    drawBackdrop(ctx);
    drawGridLines(ctx, s.dashOffset, speed || 0);
    drawYellowEdges(ctx, latShift);
    drawCorridor(ctx, left, right, laneCol, bad);
    drawCenterGuide(ctx, latShift, s.errShown, s.dashOffset);
    drawTrafficSignals(ctx, latShift, lamp);
    drawObstacles(ctx, obstacles, latShift);
    drawEgoCar(ctx);
  }, [data, lidar, speed]);

  useEffect(() => {
    let rafId;
    function loop() {
      draw();
      rafId = requestAnimationFrame(loop);
    }
    rafId = requestAnimationFrame(loop);
    // Cleanup on unmount — prevents leaked loops (note 1)
    return () => cancelAnimationFrame(rafId);
  }, [draw]);

  return (
    <canvas
      ref={canvasRef}
      width={W}
      height={H}
      className="scene"
      role="img"
      aria-label="Driving visualization"
    />
  );
}
