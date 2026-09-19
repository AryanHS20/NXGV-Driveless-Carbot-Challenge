import React from 'react';

const LAMP = {red: '#e5484d', yellow: '#f5b301', green: '#46a758'};

/** Top readout: big speed, gear roundel, traffic lamp, mode pill. */
export default function SpeedCluster({data}) {
  const speed = typeof data.speed === 'number' ? data.speed : 0;
  const lamp = String(data.traffic_light || 'unknown');
  return (
    <div className="cluster">
      <div className="speed">
        <span className="speedNum">{speed.toFixed(1)}</span>
        <span className="speedUnit">M/S</span>
      </div>
      <div className="clusterMid">
        <div className="roundel">
          <b>{typeof data.speed_pct === 'number' ? data.speed_pct : '–'}</b>
          <span>GEAR</span>
        </div>
        <div className={data.auto_mode ? 'mode auto' : 'mode'}>
          {data.auto_mode ? 'AUTO' : 'MANUAL'}
        </div>
      </div>
      <div className="lampbox">
        <div className="lamp" style={LAMP[lamp] ? {
          background: LAMP[lamp],
          boxShadow: `0 0 22px ${LAMP[lamp]}`,
        } : undefined} />
        <div className="lampname">{lamp.toUpperCase()}</div>
      </div>
    </div>
  );
}
