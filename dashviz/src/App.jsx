import React, { useRef } from 'react';
import { useTelemetry } from './api.js';
import SpeedCluster from './components/SpeedCluster.jsx';
import RoadScene from './components/RoadScene.jsx';
import TelePlots from './components/TelePlots.jsx';
import Banner from './components/Banner.jsx';

const HIST = 60;

export default function App() {
  const {data, lidar, live} = useTelemetry(500);
  const hist = useRef({speed: [], yaw: []});

  hist.current.speed = [...hist.current.speed, +data.speed || 0].slice(-HIST);
  hist.current.yaw = [...hist.current.yaw, +data.odom_yaw || 0].slice(-HIST);

  return (
    <div className="dash">
      <SpeedCluster data={data} />
      <div className="hero">
        <RoadScene data={data} lidar={lidar} />
      </div>
      <Banner data={data} />
      <TelePlots speedHist={hist.current.speed} yawHist={hist.current.yaw} />
      <div className="src">{live ? 'LIVE ROBOT DATA' : 'MOCK TELEMETRY'} · display only</div>
    </div>
  );
}
