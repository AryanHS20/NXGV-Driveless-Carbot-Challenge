import React, { useRef, useEffect, useCallback } from 'react';
import { clusterObstacles } from '../api.js';

// Base logical resolution. The canvas CSS stretches to fill the container (100% 100%),
// and these coords define the internal drawing space.
const W = 400, H = 600, CX = W / 2, HORIZON = 240, ROAD_BOT = H - 60;

function project(x, y, latShift) {
  // x is forward meters (0 to 10), y is left meters
  // More aggressive perspective narrowing for deeper 3D feel
  const t = Math.max(0, Math.min(1, 1 - x / 9)); 
  return {
    sx: CX + (y / 4.2) * 180 - (latShift || 0) * (1 - t * 0.8),
    sy: ROAD_BOT - Math.pow(t, 0.9) * (ROAD_BOT - HORIZON),
    t,
  };
}

function lanePath(side, latShift) {
  const pts = [];
  for (let i = 0; i <= 24; i++) {
    const x = 0.2 + (i / 24) * 8.5; // Look further ahead (8.5m)
    pts.push(project(x, side * (0.55 + x * 0.045), latShift));
  }
  return pts;
}

/* ── Drawing Helpers ─────────────────────────────────────────────────── */

function drawBackdrop(ctx) {
  // Atmospheric sky gradient
  const sky = ctx.createLinearGradient(0, 0, 0, HORIZON + 40);
  sky.addColorStop(0, '#e2e8f0');
  sky.addColorStop(1, '#f8fafc');
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, W, HORIZON + 40);

  // Road surface
  const road = ctx.createLinearGradient(0, HORIZON, 0, H);
  road.addColorStop(0, '#e2e8f0'); // blends with sky at horizon (fog)
  road.addColorStop(0.3, '#cbd5e1');
  road.addColorStop(1, '#94a3b8');
  ctx.fillStyle = road;

  ctx.beginPath();
  ctx.moveTo(CX - 50, HORIZON);
  ctx.lineTo(0, H);
  ctx.lineTo(W, H);
  ctx.lineTo(CX + 50, HORIZON);
  ctx.closePath();
  ctx.fill();
}

function drawGridLines(ctx, dashOffset, speed) {
  ctx.save();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
  ctx.lineWidth = 1;
  for (let i = 0; i < 9; i++) {
    const raw = ((i / 9) + (dashOffset % 60) / 60 / 9) % 1;
    const t = raw * raw; 
    const y = ROAD_BOT - t * (ROAD_BOT - HORIZON);
    const spread = 1 - Math.pow(t, 0.8) * 0.7;
    const x1 = CX - 300 * spread;
    const x2 = CX + 300 * spread;
    
    // Fade at horizon for atmospheric depth
    ctx.globalAlpha = Math.max(0, raw - 0.1); 
    ctx.beginPath();
    ctx.moveTo(x1, y);
    ctx.lineTo(x2, y);
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
  ctx.restore();
}

function strokeLane(ctx, points, color, width) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  
  // Single pass with strong shadowBlur as requested for performance
  ctx.shadowColor = color;
  ctx.shadowBlur = 18;
  
  ctx.beginPath();
  points.forEach((p, i) => {
    i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
  });
  ctx.stroke();
  ctx.restore();
}

function drawCorridor(ctx, left, right, laneCol, bad) {
  // Translucent ribbon fill
  ctx.save();
  const grad = ctx.createLinearGradient(0, HORIZON, 0, H);
  if (bad) {
    grad.addColorStop(0, 'rgba(229, 72, 77, 0.0)');
    grad.addColorStop(1, 'rgba(229, 72, 77, 0.25)');
  } else {
    grad.addColorStop(0, 'rgba(47, 124, 246, 0.0)');
    grad.addColorStop(1, 'rgba(47, 124, 246, 0.3)');
  }
  
  ctx.fillStyle = grad;
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

  // Solid boundaries
  strokeLane(ctx, left, laneCol, 5);
  strokeLane(ctx, right, laneCol, 5);
}

function drawYellowEdges(ctx, latShift) {
  [-1.9, 1.9].forEach(side => {
    const pts = lanePath(side, latShift);
    ctx.save();
    ctx.strokeStyle = '#d97706';
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.globalAlpha = 0.6;
    ctx.beginPath();
    pts.forEach((p, i) => {
      i ? ctx.lineTo(p.sx, p.sy) : ctx.moveTo(p.sx, p.sy);
    });
    ctx.stroke();
    ctx.restore();
  });
}

function drawTrafficSignals(ctx, latShift, activeLamp) {
  const lampCol = { red: '#ef4444', yellow: '#f59e0b', green: '#10b981' }[activeLamp];
  [-1, 1].forEach(side => {
    const s = project(7.5, side * 2.2, latShift); // Further down the road
    if (s.t > 0.95) return; // fade out if too close to horizon
    
    ctx.save();
    ctx.globalAlpha = 1 - s.t; // fade in from distance
    
    // Pole
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(s.sx, s.sy);
    ctx.lineTo(s.sx, s.sy - 35);
    ctx.stroke();
    
    // Housing
    ctx.fillStyle = '#1e293b';
    ctx.beginPath();
    roundRect(ctx, s.sx - 6, s.sy - 58, 12, 24, 3);
    ctx.fill();
    
    // Lamps
    ['red', 'yellow', 'green'].forEach((name, i) => {
      const active = activeLamp === name;
      ctx.fillStyle = active ? lampCol : '#334155';
      if (active) {
        ctx.shadowColor = lampCol;
        ctx.shadowBlur = 10;
      } else {
        ctx.shadowColor = 'transparent';
        ctx.shadowBlur = 0;
      }
      ctx.beginPath();
      ctx.arc(s.sx, s.sy - 52 + i * 7, 2.2, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();
  });
}

function drawObstacles3D(ctx, obstacles, latShift) {
  obstacles.forEach(o => {
    // Front bottom center of the obstacle
    const base = project(o.x, o.y, latShift);
    
    // Scale size by perspective depth (base.t)
    const scale = 1 - base.t * 0.7;
    const width = 28 * scale;
    const height = 24 * scale;
    const depth = 16 * scale; // How far back the top face goes in 2D space
    
    const fill = o.d < 0.5 ? '#dc2626' : o.d < 1.1 ? '#d97706' : '#94a3b8';
    const topFill = o.d < 0.5 ? '#f87171' : o.d < 1.1 ? '#fbbf24' : '#cbd5e1';
    const sideFill = o.d < 0.5 ? '#991b1b' : o.d < 1.1 ? '#92400e' : '#64748b';

    ctx.save();
    ctx.globalAlpha = o.n > 2 ? 0.95 : 0.65;
    
    // 1. Draw top face (rhombus/polygon projecting backwards)
    ctx.fillStyle = topFill;
    ctx.beginPath();
    ctx.moveTo(base.sx - width/2, base.sy - height); // front top left
    ctx.lineTo(base.sx + width/2, base.sy - height); // front top right
    ctx.lineTo(base.sx + width/2 * 0.85, base.sy - height - depth); // back top right
    ctx.lineTo(base.sx - width/2 * 0.85, base.sy - height - depth); // back top left
    ctx.closePath();
    ctx.fill();
    ctx.stroke(); // optional wireframe accent

    // 2. Draw front face (rectangle)
    ctx.fillStyle = fill;
    ctx.beginPath();
    roundRect(ctx, base.sx - width/2, base.sy - height, width, height, 3 * scale);
    ctx.fill();
    
    // Front face highlight/glass reflection
    ctx.fillStyle = 'rgba(255,255,255,0.2)';
    ctx.beginPath();
    ctx.moveTo(base.sx - width/2, base.sy - height);
    ctx.lineTo(base.sx + width/2, base.sy - height);
    ctx.lineTo(base.sx + width/2, base.sy - height + height * 0.4);
    ctx.lineTo(base.sx - width/2, base.sy - height + height * 0.7);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
  });
}

function drawEgoCar(ctx) {
  const w = 72, h = 120;
  const x = CX - w / 2, y = ROAD_BOT - h + 24;

  ctx.save();
  // Under-car ambient occlusion (tight, dark shadow)
  ctx.fillStyle = 'rgba(15, 23, 42, 0.4)';
  ctx.shadowColor = 'rgba(15, 23, 42, 0.3)';
  ctx.shadowBlur = 12;
  ctx.beginPath();
  roundRect(ctx, x + 8, y + h - 10, w - 16, 12, 6);
  ctx.fill();

  // Glossy road reflection (fades downward)
  const reflection = ctx.createLinearGradient(x, y + h - 10, x, y + h + 30);
  reflection.addColorStop(0, 'rgba(15, 23, 42, 0.15)');
  reflection.addColorStop(1, 'rgba(15, 23, 42, 0.0)');
  ctx.fillStyle = reflection;
  ctx.shadowBlur = 0;
  ctx.beginPath();
  roundRect(ctx, x + 4, y + h - 10, w - 8, 40, 4);
  ctx.fill();

  // Main Body
  const body = ctx.createLinearGradient(x, y, x + w, y);
  body.addColorStop(0, '#94a3b8');
  body.addColorStop(0.5, '#f8fafc');
  body.addColorStop(1, '#64748b');
  ctx.fillStyle = body;
  ctx.beginPath();
  roundRect(ctx, x, y, w, h, 18);
  ctx.fill();

  // White metallic outline
  ctx.strokeStyle = 'rgba(255,255,255,0.9)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  roundRect(ctx, x + 1, y + 1, w - 2, h - 2, 16);
  ctx.stroke();

  // Glasshouse (Roof/Windows)
  const glass = ctx.createLinearGradient(0, y + 25, 0, y + 65);
  glass.addColorStop(0, '#0f172a');
  glass.addColorStop(1, '#334155');
  ctx.fillStyle = glass;
  ctx.beginPath();
  roundRect(ctx, x + 12, y + 25, w - 24, 42, 12);
  ctx.fill();

  // Windshield gloss reflection
  ctx.fillStyle = 'rgba(255, 255, 255, 0.12)';
  ctx.beginPath();
  roundRect(ctx, x + 16, y + 28, w - 32, 14, 6);
  ctx.fill();

  // Wheels
  ctx.fillStyle = '#020617';
  [[-8, 16], [w - 4, 16], [-8, h - 24], [w - 4, h - 24]].forEach(([dx, dy]) => {
    ctx.beginPath();
    roundRect(ctx, x + dx, y + dy, 12, 18, 4);
    ctx.fill();
  });

  // Taillight bar / Headlight glow
  ctx.fillStyle = 'rgba(239, 68, 68, 0.85)';
  ctx.shadowColor = '#ef4444';
  ctx.shadowBlur = 12;
  ctx.beginPath();
  roundRect(ctx, x + 14, y + h - 8, w - 28, 4, 2);
  ctx.fill();

  ctx.restore();
}

/** Polyfill-safe roundRect */
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
    s.errShown += (err - s.errShown) * Math.min(1, dt * 6); 
    s.dashOffset += (speed || 0) * dt * 200; 

    const latShift = s.errShown * 52;
    const stale = (data.stale_streams || []).length > 0;
    const bad = stale || data.lane_lost === true;
    const laneCol = bad ? '#ef4444' : '#3b82f6';
    const lamp = String(data.traffic_light || 'unknown');

    const left = lanePath(-1, latShift);
    const right = lanePath(1, latShift);
    const obstacles = clusterObstacles(lidar || []);

    ctx.clearRect(0, 0, W, H);
    drawBackdrop(ctx);
    drawGridLines(ctx, s.dashOffset, speed || 0);
    drawYellowEdges(ctx, latShift);
    drawCorridor(ctx, left, right, laneCol, bad);
    drawTrafficSignals(ctx, latShift, lamp);
    drawObstacles3D(ctx, obstacles, latShift);
    drawEgoCar(ctx);
  }, [data, lidar, speed]);

  useEffect(() => {
    let rafId;
    function loop() {
      draw();
      rafId = requestAnimationFrame(loop);
    }
    rafId = requestAnimationFrame(loop);
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
