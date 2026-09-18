(() => {
  'use strict';
  const replayName = new URLSearchParams(window.location.search).get('replay');
  if (!replayName) return;
  const safeName = /^[A-Za-z0-9_.-]+\.json$/.test(replayName);
  const panel = document.createElement('section');
  panel.id = 'replay-viewer';
  panel.innerHTML = `<div class="replay-head"><div><small>RECORDED ROBOT RUN</small><b id="replay-title">Loading…</b></div><div class="replay-controls"><button id="replay-play" disabled>Play</button><button id="replay-step" disabled>Step</button><label>Replay speed <select id="replay-rate"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select></label></div></div><canvas id="replay-canvas" width="1100" height="380"></canvas><div class="replay-readout"><span id="replay-time">0.0 s</span><span id="replay-pose">Pose unavailable</span><span id="replay-lane">Lane —</span><span id="replay-light">Light —</span><span id="replay-events">No events</span></div>`;
  document.querySelector('main').prepend(panel);
  const style = document.createElement('style');
  style.textContent = `#replay-viewer{margin:14px 18px;padding:14px;border:1px solid #29465c;border-radius:12px;background:#0d1b28;color:#e9f5fb}.replay-head,.replay-controls,.replay-readout{display:flex;gap:12px;align-items:center;justify-content:space-between;flex-wrap:wrap}.replay-head b{display:block;color:#47d9f5}.replay-controls label{font-size:12px}#replay-canvas{display:block;width:100%;height:380px;margin:12px 0;background:#07111a;border-radius:8px}.replay-readout{justify-content:flex-start;font:12px/1.4 monospace}.replay-readout span{padding:5px 9px;background:#142838;border-radius:6px}#replay-events{color:#ffd166}`;
  document.head.appendChild(style);
  const canvas = document.getElementById('replay-canvas');
  const context = canvas.getContext('2d');
  const playButton = document.getElementById('replay-play');
  const stepButton = document.getElementById('replay-step');
  const rateSelect = document.getElementById('replay-rate');
  let frames = [], index = 0, playing = false, timer = null;
  let bounds = {minX: -1, maxX: 1, minY: -1, maxY: 1};
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  function cleanFrame(frame) {
    if (!frame || !finite(frame.t)) return null;
    const odom = frame.odom && finite(frame.odom.x) && finite(frame.odom.y) && finite(frame.odom.yaw) ? {x: frame.odom.x, y: frame.odom.y, yaw: frame.odom.yaw} : null;
    const corridor = Array.isArray(frame.corridor) ? frame.corridor.filter(point => point && finite(point.forward_m) && finite(point.left_m)).map(point => ({forward_m: point.forward_m, left_m: point.left_m, width_m: finite(point.width_m) ? point.width_m : 0})) : [];
    return {t: frame.t, odom, corridor, speed: finite(frame.speed) ? frame.speed : 0, lane_error: finite(frame.lane_error) ? frame.lane_error : null, lane_lost: frame.lane_lost === true, tl: typeof frame.tl === 'string' ? frame.tl : 'unknown', hill: frame.hill === true, tunnel: frame.tunnel === true};
  }
  function computeBounds() {
    const poses = frames.filter(frame => frame.odom).map(frame => frame.odom);
    if (!poses.length) return;
    bounds = {minX: Math.min(...poses.map(p => p.x)), maxX: Math.max(...poses.map(p => p.x)), minY: Math.min(...poses.map(p => p.y)), maxY: Math.max(...poses.map(p => p.y))};
    const pad = Math.max(0.5, 0.12 * Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY));
    bounds.minX -= pad; bounds.maxX += pad; bounds.minY -= pad; bounds.maxY += pad;
  }
  function project(x, y) {
    const scale = Math.min((canvas.width - 40) / Math.max(0.01, bounds.maxX - bounds.minX), (canvas.height - 40) / Math.max(0.01, bounds.maxY - bounds.minY));
    return [20 + (x - bounds.minX) * scale, canvas.height - 20 - (y - bounds.minY) * scale];
  }
  function draw() {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.strokeStyle = '#173247'; context.lineWidth = 1;
    for (let x = 0; x < canvas.width; x += 50) { context.beginPath(); context.moveTo(x, 0); context.lineTo(x, canvas.height); context.stroke(); }
    for (let y = 0; y < canvas.height; y += 50) { context.beginPath(); context.moveTo(0, y); context.lineTo(canvas.width, y); context.stroke(); }
    const history = frames.slice(0, index + 1).filter(frame => frame.odom);
    if (history.length) { context.strokeStyle = '#47d9f5'; context.lineWidth = 3; context.beginPath(); history.forEach((frame, i) => { const p = project(frame.odom.x, frame.odom.y); i ? context.lineTo(p[0], p[1]) : context.moveTo(p[0], p[1]); }); context.stroke(); }
    const frame = frames[index];
    if (!frame) return;
    if (frame.odom) {
      const pose = frame.odom;
      frame.corridor.forEach(point => { const x = pose.x + point.forward_m * Math.cos(pose.yaw) - point.left_m * Math.sin(pose.yaw); const y = pose.y + point.forward_m * Math.sin(pose.yaw) + point.left_m * Math.cos(pose.yaw); const p = project(x, y); context.fillStyle = '#7ddc87'; context.beginPath(); context.arc(p[0], p[1], 4, 0, Math.PI * 2); context.fill(); });
      const p = project(pose.x, pose.y); context.save(); context.translate(p[0], p[1]); context.rotate(-pose.yaw); context.fillStyle = '#ffd166'; context.beginPath(); context.moveTo(13, 0); context.lineTo(-9, -7); context.lineTo(-9, 7); context.closePath(); context.fill(); context.restore();
    }
    document.getElementById('replay-time').textContent = `${(frame.t - frames[0].t).toFixed(1)} s / frame ${index + 1} of ${frames.length}`;
    document.getElementById('replay-pose').textContent = frame.odom ? `Pose ${frame.odom.x.toFixed(2)}, ${frame.odom.y.toFixed(2)}, ${(frame.odom.yaw * 180 / Math.PI).toFixed(1)}°` : 'Pose unavailable';
    document.getElementById('replay-lane').textContent = frame.lane_error === null ? 'Lane —' : `Lane ${(frame.lane_error * 100).toFixed(1)} cm${frame.lane_lost ? ' LOST' : ''}`;
    document.getElementById('replay-light').textContent = `Light ${frame.tl}`;
    const events = [frame.hill && 'hill', frame.tunnel && 'tunnel', frame.lane_lost && 'lane lost'].filter(Boolean);
    document.getElementById('replay-events').textContent = events.length ? events.join(' · ') : 'No active events';
  }
  function schedule() {
    clearTimeout(timer);
    if (!playing || index >= frames.length - 1) { playing = false; playButton.textContent = 'Play'; return; }
    const delta = Math.max(20, Math.min(1000, (frames[index + 1].t - frames[index].t) * 1000 / Number(rateSelect.value)));
    timer = setTimeout(() => { index += 1; draw(); schedule(); }, delta);
  }
  playButton.addEventListener('click', () => { playing = !playing; playButton.textContent = playing ? 'Pause' : 'Play'; schedule(); });
  stepButton.addEventListener('click', () => { playing = false; playButton.textContent = 'Play'; index = Math.min(frames.length - 1, index + 1); draw(); });
  rateSelect.addEventListener('change', schedule);
  if (!safeName) { document.getElementById('replay-title').textContent = 'Invalid replay filename'; return; }
  fetch(`/api/replay/get?name=${encodeURIComponent(replayName)}`).then(response => { if (!response.ok) throw new Error(`HTTP ${response.status}`); return response.json(); }).then(payload => {
    if (!payload || payload.version !== 1 || !Array.isArray(payload.frames)) throw new Error('unsupported replay schema');
    frames = payload.frames.map(cleanFrame).filter(Boolean).sort((a, b) => a.t - b.t);
    if (!frames.length) throw new Error('replay contains no valid frames');
    document.getElementById('replay-title').textContent = `${replayName} · ${frames.length} frames`;
    playButton.disabled = false; stepButton.disabled = false; computeBounds(); draw();
  }).catch(error => { document.getElementById('replay-title').textContent = `Replay failed: ${error.message}`; });
})();
