import React, { useRef } from 'react';
import { useTelemetry } from './api.js';
import SpeedCluster from './components/SpeedCluster.jsx';
import RoadScene from './components/RoadScene.jsx';
import TelePlots from './components/TelePlots.jsx';
import Banner from './components/Banner.jsx';

const HIST = 60;

export default function App() {
  const { data, lidar, live } = useTelemetry(500);
  const hist = useRef({ speed: [], yaw: [], err: [] });

  hist.current.speed = [...hist.current.speed, +data.speed || 0].slice(-HIST);
  hist.current.yaw = [...hist.current.yaw, +data.odom_yaw || 0].slice(-HIST);
  hist.current.err = [...hist.current.err, +data.lane_error || 0].slice(-HIST);

  return (
    <div className="dash">
      {/* Background Canvas Layer */}
      <div className="hero">
        <RoadScene data={data} lidar={lidar} speed={+data.speed || 0} />
      </div>
      
      {/* Floating HUD Layer */}
      <div className="hud">
        <SpeedCluster data={data} />
        
        <div className="bottomHUD">
          <Banner data={data} />
          <TelePlots
            speedHist={hist.current.speed}
            yawHist={hist.current.yaw}
            errHist={hist.current.err}
          />
          <div className="src">{live ? 'LIVE ROBOT DATA' : 'MOCK TELEMETRY'} · display only</div>
        </div>
      </div>
    </div>
  );
}
