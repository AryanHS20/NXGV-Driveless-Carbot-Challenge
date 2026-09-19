import React from 'react';

function Spark({values, min, max, stroke}) {
  const W = 180, H = 44;
  const pts = values.map((v, i) => {
    const x = (i / Math.max(1, values.length - 1)) * W;
    const t = Math.max(0, Math.min(1, (v - min) / Math.max(1e-9, max - min)));
    return `${x.toFixed(1)},${(H - 4 - t * (H - 8)).toFixed(1)}`;
  }).join(' ');
  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="spark">
      <polyline points={pts} fill="none" stroke={stroke} strokeWidth="2"
        strokeLinejoin="round" strokeLinecap="round" />
    </svg>
  );
}

/** Speed + yaw history sparklines (Tesla engineering-UI style). */
export default function TelePlots({speedHist, yawHist}) {
  return (
    <div className="plots">
      <div className="plot">
        <span>Vehicle speed</span>
        <Spark values={speedHist} min={0} max={Math.max(0.3, ...speedHist)} stroke="#2f7cf6" />
      </div>
      <div className="plot">
        <span>Yaw rate</span>
        <Spark values={yawHist} min={-0.6} max={0.6} stroke="#8b98ac" />
      </div>
    </div>
  );
}
