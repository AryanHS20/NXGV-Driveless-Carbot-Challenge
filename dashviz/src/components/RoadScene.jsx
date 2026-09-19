import React, { useMemo } from 'react';
import { clusterObstacles } from '../api.js';

const W = 400, H = 560, CX = W / 2, HORIZON = 150, ROAD_BOT = H - 64;

/** Ground (x fwd, y left, metres) to scene pixels. */
function project(x, y, latShift) {
  const t = Math.max(0, Math.min(1, 1 - x / 7));
  return {
    x: CX + (y / 4.2) * 150 - (latShift || 0) * (1 - t * 0.75),
    y: ROAD_BOT - t * (ROAD_BOT - HORIZON),
  };
}

function lanePoints(side, latShift) {
  const pts = [];
  for (let i = 0; i <= 24; i++) {
    const x = 0.2 + (i / 24) * 6.5;
    pts.push(project(x, side * (0.55 + x * 0.045), latShift));
  }
  return pts;
}

const toPath = (pts) => 'M' + pts.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' L');

/** Tesla-white vector scene: corridor ribbon, ego car, obstacles, signals. */
export default function RoadScene({data, lidar}) {
  const err = Math.max(-1, Math.min(1, +data.lane_error || 0));
  const latShift = err * 52;
  const bad = (data.stale_streams || []).length > 0 || data.lane_lost === true;
  const laneCol = bad ? '#e5484d' : '#2f7cf6';

  const obstacles = useMemo(() => clusterObstacles(lidar || []), [lidar]);
  const lamp = String(data.traffic_light || 'unknown');
  const lampCol = {red: '#e5484d', yellow: '#f5b301', green: '#46a758'}[lamp];

  const left = lanePoints(-1, latShift);
  const right = lanePoints(1, latShift);
  const ribbon = [...left].reverse().concat(right);

  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="scene" role="img" aria-label="Driving visualization">
      <defs>
        <linearGradient id="road" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#f4f6f9" />
          <stop offset="1" stopColor="#dde3ec" />
        </linearGradient>
        <linearGradient id="carbody" x1="0" y1="0" x2="1" y2="0">
          <stop offset="0" stopColor="#9aa7ba" />
          <stop offset="0.5" stopColor="#eef2f7" />
          <stop offset="1" stopColor="#8b98ac" />
        </linearGradient>
        <filter id="soft" x="-40%" y="-40%" width="180%" height="180%">
          <feGaussianBlur stdDeviation="5" />
        </filter>
      </defs>

      <rect x="0" y="0" width={W} height={H} fill="#eef1f6" />
      <polygon points={`0,${H} 0,${HORIZON} ${W},${HORIZON} ${W},${H}`} fill="url(#road)" />
      <line x1="0" y1={HORIZON} x2={W} y2={HORIZON} stroke="#c7cfdb" strokeWidth="1.5" />

      {/* corridor ribbon */}
      <polygon points={ribbon.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ')}
        fill="rgba(47,124,246,0.16)" />
      {/* lane boundaries */}
      <path d={toPath(left)} fill="none" stroke={laneCol} strokeWidth="5"
        strokeLinecap="round" opacity="0.9" />
      <path d={toPath(right)} fill="none" stroke={laneCol} strokeWidth="5"
        strokeLinecap="round" opacity="0.9" />
      {/* yellow left edge */}
      <path d={toPath(lanePoints(-1.9, latShift))} fill="none" stroke="#e3b008"
        strokeWidth="3" strokeLinecap="round" opacity="0.8" />

      {/* traffic signal poles */}
      {[-1, 1].map(side => {
        const s = project(5.5, side * 2.2, latShift);
        return (
          <g key={side}>
            <line x1={s.x} y1={s.y} x2={s.x} y2={s.y - 42} stroke="#9aa3b2" strokeWidth="3" />
            <rect x={s.x - 7} y={s.y - 58} width="14" height="18" rx="3" fill="#2b3442" />
            <circle cx={s.x} cy={s.y - 49} r="4.5"
              fill={lampCol || '#5b6472'}
              filter={lampCol ? 'url(#soft)' : undefined} />
          </g>
        );
      })}

      {/* obstacles as vehicle blocks */}
      {obstacles.map((o, i) => {
        const s = project(o.x, o.y, latShift);
        const size = 10 + (1 - Math.min(1, o.d / 6)) * 26;
        const fill = o.d < 0.5 ? '#e5484d' : o.d < 1.1 ? '#f5b301' : '#b9c2cf';
        return (
          <g key={i} opacity={o.n > 2 ? 1 : 0.55}>
            <rect x={s.x - size / 2} y={s.y - size * 1.15} width={size}
              height={size * 1.15} rx={size * 0.22} fill={fill} />
            <rect x={s.x - size * 0.32} y={s.y - size * 0.95} width={size * 0.64}
              height={size * 0.4} rx={size * 0.12} fill="rgba(20,28,40,0.55)" />
          </g>
        );
      })}

      {/* ego car */}
      <g>
        <rect x={CX - 32} y={ROAD_BOT - 96} width="64" height="108" rx="15" fill="rgba(20,30,45,0.18)" />
        <rect x={CX - 32} y={ROAD_BOT - 100} width="64" height="108" rx="15" fill="url(#carbody)"
          stroke="#ffffff" strokeWidth="1.5" />
        <rect x={CX - 22} y={ROAD_BOT - 80} width="44" height="38" rx="10" fill="#101c2c" />
        <rect x={CX - 18} y={ROAD_BOT - 76} width="36" height="12" rx="6" fill="rgba(140,200,255,0.35)" />
        {[[-38, -88], [26, -88], [-38, -6], [26, -6]].map(([dx, dy], i) => (
          <rect key={i} x={CX + dx} y={ROAD_BOT + dy} width="12" height="16" rx="4" fill="#141a24" />
        ))}
        <rect x={CX - 20} y={ROAD_BOT + 1} width="40" height="4" rx="2" fill="#bfe1ff" />
      </g>
    </svg>
  );
}
