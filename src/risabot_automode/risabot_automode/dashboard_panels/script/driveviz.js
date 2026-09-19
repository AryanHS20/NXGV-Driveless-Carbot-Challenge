let drivevizOn = false;
let drivevizData = null;
let drivevizLidar = [];
let drivevizTimer = 0;

function toggleDriveviz() {
  drivevizOn = !drivevizOn;
  document.getElementById('drivevizPanel').style.display = drivevizOn ? 'block' : 'none';
  document.getElementById('drivevizBtn').textContent = drivevizOn ? '🚗 Disable Drive Viz' : '🚗 Enable Drive Viz';
  if (drivevizOn) {
    drivevizTick();
    drivevizTimer = setInterval(drivevizTick, 500);
  } else {
    clearInterval(drivevizTimer);
  }
}

function drivevizTick() {
  fetch('/data').then(r => r.json()).then(d => {
    drivevizData = d;
    const mode = document.getElementById('drivevizMode');
    if (mode) mode.textContent = d.auto_mode ? 'AUTO' : 'MANUAL';
    drawDriveviz();
  }).catch(() => {});
  fetch('/lidar_data').then(r => r.json()).then(d => {
    drivevizLidar = d.points || [];
    drawDriveviz();
  }).catch(() => {});
}

function drivevizPt(p) {
  if (Array.isArray(p)) return {x: p[0], y: p[1]};
  return {x: p.x || 0, y: p.y || 0};
}

function drawDriveviz() {
  const canvas = document.getElementById('drivevizCanvas');
  if (!canvas || !drivevizOn) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const d = drivevizData || {};
  const err = (typeof d.lane_error === 'number') ? d.lane_error : 0;
  const stale = [].concat(d.stale_streams || [], d.health_stale || []);
  const camBad = stale.indexOf('camera_frame') >= 0 || stale.indexOf('lane_error') >= 0;
  const lidarBad = stale.indexOf('scan') >= 0;
  const laneLost = d.lane_lost === true;
  const estate = String(d.state || '');
  const estop = estate === 'EMERGENCY_STOP';

  ctx.clearRect(0, 0, W, H);

  // ── Top readout: big speed + gear roundel + traffic lamp ──
  const speed = (typeof d.speed === 'number') ? d.speed : 0;
  ctx.fillStyle = '#1a2332';
  ctx.textAlign = 'left';
  ctx.font = '800 44px Inter, sans-serif';
  ctx.fillText(speed.toFixed(1), 14, 52);
  ctx.font = '600 13px Inter, sans-serif';
  ctx.fillStyle = '#6b7a90';
  ctx.fillText('m/s', 16, 70);
  const pct = (typeof d.speed_pct === 'number') ? d.speed_pct : null;
  if (pct !== null) {
    ctx.strokeStyle = '#1e66f5';
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(118, 40, 20, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#1e66f5';
    ctx.font = '800 15px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(String(pct), 118, 45);
    ctx.font = '600 8px Inter, sans-serif';
    ctx.fillText('GEAR', 118, 55);
    ctx.textAlign = 'left';
  }
  const lamp = String(d.traffic_light || 'unknown');
  const lampColors = {red: '#e5484d', yellow: '#f5b301', green: '#46a758'};
  ctx.fillStyle = lampColors[lamp] || '#b0b8c5';
  ctx.beginPath(); ctx.arc(W - 26, 38, 12, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#6b7a90';
  ctx.font = '600 10px Inter, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(lamp.toUpperCase(), W - 26, 60);
  ctx.textAlign = 'left';

  // ── Perspective road ──
  const horizon = 120, roadTop = 150, roadBot = H - 70;
  const cx = W / 2;
  const shift = Math.max(-1, Math.min(1, err)) * 46;
  const laneColor = (camBad || laneLost) ? '#e5484d' : '#1e66f5';
  ctx.strokeStyle = laneColor;
  ctx.lineWidth = 5;
  ctx.lineCap = 'round';
  // Left and right lane boundaries converging to the vanishing point.
  [[-64, -10], [64, 10]].forEach(function(side) {
    const xBase = side[0], xTop = side[1];
    ctx.beginPath();
    ctx.moveTo(cx + xBase - shift, roadBot);
    ctx.lineTo(cx + xTop - shift * 0.25, roadTop);
    ctx.stroke();
  });
  // Faint road edges for context.
  ctx.strokeStyle = 'rgba(30,102,245,0.18)';
  ctx.lineWidth = 2;
  [[-104, -22], [104, 22]].forEach(function(side) {
    ctx.beginPath();
    ctx.moveTo(cx + side[0] - shift, roadBot);
    ctx.lineTo(cx + side[1] - shift * 0.25, roadTop);
    ctx.stroke();
  });
  ctx.fillStyle = '#8a94a6';
  ctx.font = '600 9px Inter, sans-serif';
  ctx.fillText('horizon', cx + 74, horizon + 12);

  // ── Obstacles from LiDAR (x forward, y left), near ones as blocks ──
  if (!lidarBad) {
    drivevizLidar.forEach(function(raw) {
      const p = drivevizPt(raw);
      if (p.x < -0.5 || p.x > 6 || Math.abs(p.y) > 4) return;
      const t = Math.max(0, Math.min(1, 1 - p.x / 6));
      const sx = cx + (p.y / 4) * 130 - shift * (1 - t);
      const sy = roadBot - t * (roadBot - roadTop);
      const dist = Math.sqrt(p.x * p.x + p.y * p.y);
      if (dist < 1.0) {
        ctx.fillStyle = dist < 0.4 ? '#e5484d' : '#f5b301';
        const s = 10 + (1 - dist) * 14;
        ctx.beginPath();
        ctx.roundRect(sx - s / 2, sy - s, s, s, 3);
        ctx.fill();
      } else {
        ctx.fillStyle = 'rgba(70,167,88,0.75)';
        ctx.beginPath(); ctx.arc(sx, sy, 2.5, 0, Math.PI * 2); ctx.fill();
      }
    });
  }

  // ── Car avatar (top-down, Tesla blue) ──
  const carW = 56, carH = 96, carX = cx - carW / 2, carY = roadBot - carH + 14;
  ctx.fillStyle = 'rgba(0,0,0,0.12)';
  ctx.beginPath();
  ctx.roundRect(carX + 3, carY + 5, carW, carH, 12);
  ctx.fill();
  ctx.fillStyle = '#2f6fed';
  ctx.beginPath();
  ctx.roundRect(carX, carY, carW, carH, 12);
  ctx.fill();
  ctx.fillStyle = '#101c2c';
  ctx.beginPath();
  ctx.roundRect(carX + 8, carY + 18, carW - 16, 30, 8);
  ctx.fill();
  ctx.fillStyle = '#0b0e13';
  [[-6, 8], [carW - 6, 8], [-6, carH - 14], [carW - 6, carH - 14]].forEach(function(w) {
    ctx.beginPath();
    ctx.roundRect(carX + w[0], carY + w[1], 12, 7, 2);
    ctx.fill();
  });

  // ── Warning banner (Tesla-style hold-to-take-over pill) ──
  let banner = '';
  let bannerRed = false;
  if (estop) {
    banner = 'EMERGENCY STOP';
    bannerRed = true;
  } else if (camBad) {
    banner = 'Take over — camera stale';
    bannerRed = true;
  } else if (lidarBad) {
    banner = 'Take over — LiDAR stale';
    bannerRed = true;
  } else if (laneLost) {
    banner = 'Lane lost — be ready to take over';
  } else if (stale.length > 0) {
    banner = 'Degraded — ' + stale.slice(0, 2).join(', ');
  }
  if (banner) {
    ctx.font = '600 12px Inter, sans-serif';
    const tw = ctx.measureText(banner).width + 28;
    const bx = cx - tw / 2, by = carY - 34;
    ctx.fillStyle = bannerRed ? '#c81e2b' : '#101c2c';
    ctx.beginPath();
    ctx.roundRect(bx, by, tw, 24, 12);
    ctx.fill();
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText(banner, cx, by + 16);
    ctx.textAlign = 'left';
  }
}
