import React from 'react';

function Spark({ values, min, max, stroke, label }) {
  const W = 180, H = 48;
  const pts = values.map((v, i) => {
    const x = (i / Math.max(1, values.length - 1)) * W;
    const t = Math.max(0, Math.min(1, (v - min) / Math.max(1e-9, max - min)));
    return `${x.toFixed(1)},${(H - 6 - t * (H - 12)).toFixed(1)}`;
  }).join(' ');

  // Current value for the readout
  const current = values.length > 0 ? values[values.length - 1] : 0;

  return (
    <div className="plot">
      <div className="plotHeader">
        <span className="plotLabel">{label}</span>
        <span className="plotValue" style={{ color: stroke }}>{current.toFixed(3)}</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} className="spark">
        {/* Subtle zero line for signed values */}
        {min < 0 && (
          <line
            x1="0" y1={(H - 6 - (0 - min) / (max - min) * (H - 12)).toFixed(1)}
            x2={W} y2={(H - 6 - (0 - min) / (max - min) * (H - 12)).toFixed(1)}
            stroke="rgba(140,155,185,0.2)" strokeWidth="1" strokeDasharray="4,4"
          />
        )}
        <polyline points={pts} fill="none" stroke={stroke} strokeWidth="2"
          strokeLinejoin="round" strokeLinecap="round" />
      </svg>
    </div>
  );
}

/** Speed + yaw + lane_error history sparklines (Tesla engineering-UI style). */
export default function TelePlots({ speedHist, yawHist, errHist }) {
  return (
    <div className="plots">
      <Spark values={speedHist} min={0} max={Math.max(0.3, ...speedHist)}
        stroke="#2f7cf6" label="SPEED" />
      <Spark values={yawHist} min={-0.6} max={0.6}
        stroke="#8b98ac" label="YAW" />
      <Spark values={errHist || []} min={-1} max={1}
        stroke="#22a3a8" label="LANE ERR" />
    </div>
  );
}
