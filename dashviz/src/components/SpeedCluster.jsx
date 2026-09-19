import React from 'react';

const LAMP = { red: '#e5484d', yellow: '#f5b301', green: '#46a758' };

/** Inline SVG steering wheel that rotates with lane error. */
function SteeringWheel({ laneError }) {
  const angle = Math.max(-35, Math.min(35, (laneError || 0) * 35));
  return (
    <svg
      viewBox="0 0 36 36"
      className="steeringWheel"
      style={{ transform: `rotate(${angle.toFixed(1)}deg)` }}
      aria-label={`Steering: ${angle.toFixed(0)}°`}
    >
      {/* Outer ring */}
      <circle cx="18" cy="18" r="15" fill="none" stroke="#5a6577" strokeWidth="3.5"
        strokeLinecap="round" />
      {/* Grip highlights */}
      <path d="M3.5 18 A14.5 14.5 0 0 1 7 9" fill="none" stroke="#3d4a5c" strokeWidth="3"
        strokeLinecap="round" />
      <path d="M32.5 18 A14.5 14.5 0 0 0 29 9" fill="none" stroke="#3d4a5c" strokeWidth="3"
        strokeLinecap="round" />
      {/* Spokes */}
      <line x1="8" y1="20" x2="14" y2="19" stroke="#5a6577" strokeWidth="2" strokeLinecap="round" />
      <line x1="28" y1="20" x2="22" y2="19" stroke="#5a6577" strokeWidth="2" strokeLinecap="round" />
      <line x1="18" y1="33" x2="18" y2="23" stroke="#5a6577" strokeWidth="2" strokeLinecap="round" />
      {/* Center hub */}
      <circle cx="18" cy="19" r="4.5" fill="#4a5568" />
      <circle cx="18" cy="19" r="2" fill="#6b7a90" />
    </svg>
  );
}

/** Top readout: big speed, gear roundel, steering wheel, traffic lamp, mode pill. */
export default function SpeedCluster({ data }) {
  const speed = typeof data.speed === 'number' ? data.speed : 0;
  const lamp = String(data.traffic_light || 'unknown');

  return (
    <div className="cluster">
      {/* Left: speed */}
      <div className="speed">
        <span className="speedNum">{speed.toFixed(1)}</span>
        <span className="speedUnit">M/S</span>
      </div>

      {/* Center: gear roundel + mode pill + steering wheel */}
      <div className="clusterMid">
        <div className="roundel">
          <b>{typeof data.speed_pct === 'number' ? data.speed_pct : '–'}</b>
          <span>MAX</span>
        </div>
        <SteeringWheel laneError={data.lane_error} />
        <div className={data.auto_mode ? 'mode auto' : 'mode'}>
          {data.auto_mode ? 'AUTO' : 'MANUAL'}
        </div>
      </div>

      {/* Right: traffic lamp */}
      <div className="lampbox">
        <div
          className="lamp"
          style={LAMP[lamp] ? {
            background: LAMP[lamp],
            boxShadow: `0 0 22px ${LAMP[lamp]}`,
            borderColor: 'transparent',
          } : undefined}
        />
        <div className="lampname">{lamp.toUpperCase()}</div>
      </div>

      {/* State label */}
      <div className="stateLabel">{data.state || '—'}</div>
    </div>
  );
}
