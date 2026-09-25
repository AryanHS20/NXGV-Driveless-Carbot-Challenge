/* Interactive offline harness for the upstream ROS parking phase controller. */
(() => {
  'use strict';
  const core = window.UpstreamParkingCore;
  const root = document.getElementById('upstream-parking');
  if (!root || !core) return;
  const $ = id => document.getElementById(id);
  const canvas = $('upstream-canvas');
  const ctx = canvas.getContext('2d');
  const kind = $('upstream-kind');
  const scrub = $('upstream-scrub');
  const play = $('upstream-play');
  const rate = $('upstream-rate');
  const status = $('upstream-status');
  const result = $('upstream-result');
  const poseInputs = ['upstream-x', 'upstream-y', 'upstream-heading', 'upstream-wheel', 'upstream-depth'];
  let trial, index = 0, playing = false, lastFrame = 0, accumulated = 0;

  function configure() {
    try {
      trial = core.simulate(kind.value, {
        start: {x: Number($('upstream-x').value), y: Number($('upstream-y').value),
          yawDeg: Number($('upstream-heading').value)},
        maxWheelDeg: Number($('upstream-wheel').value),
        bayDepth: Number($('upstream-depth').value),
      });
    } catch (error) {
      playing = false;
      play.textContent = 'Play controller';
      result.textContent = `Check the start pose and wheel-steer inputs: ${error.message}`;
      return;
    }
    playing = false;
    accumulated = 0;
    index = 0;
    scrub.max = String(trial.samples.length - 1);
    scrub.value = '0';
    play.textContent = 'Play controller';
    const parked = trial.parked;
    const turn = trial.samples.find(s => s.phase === 'PERP_FORWARD');
    const turnDeg = turn ? (turn.yaw - trial.samples[0].yaw) * 180 / Math.PI : 0;
    result.textContent = `${trial.complete ? 'Sequence completed' : 'Sequence did not finish'} in ${trial.duration.toFixed(2)} s. ` +
      `At parking wait: ${parked?.fits ? 'BODY INSIDE BAY' : 'BODY OUTSIDE BAY'}. ` +
      (trial.startsInside ? 'Start footprint was already inside the bay. ' : '') +
      (trial.kind === 'perpendicular' ? `Turn-in heading change: ${turnDeg.toFixed(1)}° (script requests 90°). ` : '') +
      'The script then waits 3 s and commands an exit.';
    render();
  }

  function moveTo(i) {
    index = Math.max(0, Math.min(trial.samples.length - 1, Math.floor(i)));
    scrub.value = String(index);
    render();
  }

  function drawCar(pose, project, alpha = 1) {
    const vertices = core.corners(pose).map(project);
    ctx.fillStyle = `rgba(78,217,237,${0.28 * alpha})`;
    ctx.strokeStyle = `rgba(206,246,255,${alpha})`;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(...vertices[0]);
    for (let i = 1; i < vertices.length; i++) ctx.lineTo(...vertices[i]);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    const [x, y] = project(pose);
    const [frontX, frontY] = project({x: pose.x + 0.19 * Math.cos(pose.yaw),
      y: pose.y + 0.19 * Math.sin(pose.yaw)});
    ctx.strokeStyle = `rgba(255,220,110,${alpha})`;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(frontX, frontY); ctx.stroke();
  }

  function render() {
    const width = Math.max(320, Math.round(canvas.clientWidth));
    const height = Math.max(300, Math.round(canvas.clientHeight));
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0c1b29'; ctx.fillRect(0, 0, width, height);
    const areaH = height - 118;
    const all = trial.samples;
    const points = all.map(s => ({x: s.x, y: s.y}));
    const x0 = Math.min(-0.15, ...points.map(p => p.x), trial.bay.x0) - 0.12;
    const x1 = Math.max(0.85, ...points.map(p => p.x), trial.bay.x1) + 0.12;
    const y0 = Math.min(-0.35, ...points.map(p => p.y), trial.bay.y0) - 0.12;
    const y1 = Math.max(0.50, ...points.map(p => p.y), trial.bay.y1) + 0.12;
    const scale = Math.min((width - 70) / (x1 - x0), (areaH - 50) / (y1 - y0));
    const left = (width - (x1 - x0) * scale) / 2;
    const top = (areaH - (y1 - y0) * scale) / 2;
    const project = p => [left + (p.x - x0) * scale, top + (y1 - p.y) * scale];

    // Bay is open at y=0; show only the three solid edges.
    const bay = trial.bay;
    const a = project({x: bay.x0, y: bay.y0}), b = project({x: bay.x0, y: bay.y1});
    const c = project({x: bay.x1, y: bay.y1}), d = project({x: bay.x1, y: bay.y0});
    ctx.fillStyle = '#183942';
    ctx.fillRect(b[0], b[1], c[0] - b[0], a[1] - b[1]);
    ctx.strokeStyle = '#ffdc6e'; ctx.lineWidth = 4;
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.lineTo(...c); ctx.lineTo(...d); ctx.stroke();
    ctx.setLineDash([6, 5]); ctx.strokeStyle = '#64798b'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...d); ctx.stroke(); ctx.setLineDash([]);
    ctx.font = '12px system-ui'; ctx.fillStyle = '#ffdc6e';
    ctx.fillText(`${trial.kind.toUpperCase()} BAY ${Math.round((bay.x1 - bay.x0) * 100)} × ${Math.round((bay.y1 - bay.y0) * 100)} cm`, b[0] + 8, b[1] + 17);
    ctx.fillStyle = '#9bb6c7'; ctx.fillText('OPENING', a[0] + 8, a[1] + 18);

    // Draw completed travel by command direction. Ghost shows the stop at WAIT.
    for (let i = 1; i <= index; i++) {
      const prev = all[i - 1], cur = all[i];
      if (!cur.linear) continue;
      ctx.strokeStyle = cur.linear > 0 ? '#4ce5a0' : '#ffac6b'; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(...project(prev)); ctx.lineTo(...project(cur)); ctx.stroke();
    }
    if (trial.parked && trial.parked.time <= all[index].time) drawCar(trial.parked.pose, project, 0.45);
    drawCar(all[index], project);
    ctx.fillStyle = '#a9bed0';
    ctx.fillText('Plant: 21 cm wheelbase · 27.5 × 18.5 cm body · ideal speed/odom · provisional steering', 15, 19);

    const timelineTop = areaH + 15, timelineH = 34;
    const startX = 22, timelineW = width - 44;
    const xAt = time => startX + time / trial.duration * timelineW;
    const colors = {PARALLEL_FORWARD:'#397d9e', PARALLEL_STEER_REVERSE:'#ba6b45',
      PARALLEL_STRAIGHTEN:'#3c8d78', PARALLEL_WAIT:'#776aa7', PARALLEL_EXIT:'#3c8d78',
      PERP_TURN_IN:'#a76e54', PERP_FORWARD:'#397d9e', PERP_WAIT:'#776aa7',
      PERP_REVERSE_OUT:'#ba6b45'};
    for (let i = 0; i < trial.transitions.length - 1; i++) {
      const entry = trial.transitions[i], next = trial.transitions[i + 1];
      ctx.fillStyle = colors[entry.phase] || '#445869';
      ctx.fillRect(xAt(entry.time), timelineTop, Math.max(1, xAt(next.time) - xAt(entry.time)), timelineH);
      if (xAt(next.time) - xAt(entry.time) > 95) {
        ctx.fillStyle = '#f7fbff'; ctx.font = '11px monospace';
        ctx.fillText(entry.phase.replace('PARALLEL_', '').replace('PERP_', ''), xAt(entry.time) + 5, timelineTop + 21);
      }
    }
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(xAt(all[index].time), timelineTop - 5);
    ctx.lineTo(xAt(all[index].time), timelineTop + timelineH + 5); ctx.stroke();
    ctx.fillStyle = '#a9bed0'; ctx.font = '11px monospace';
    ctx.fillText('0 s', startX, height - 29);
    ctx.fillText(`${trial.duration.toFixed(2)} s`, width - 85, height - 29);
    ctx.fillText('Green path: forward · Orange path: reverse · Faded car: parking wait', 15, height - 8);

    const s = all[index];
    status.textContent = `${s.time.toFixed(2)} s / ${trial.duration.toFixed(2)} s · ${s.phase} · ` +
      `linear.x ${s.linear.toFixed(3)} m/s · angular.z ${s.angular.toFixed(2)} · ` +
      `heading ${(s.yaw * 180 / Math.PI).toFixed(1)}° · odom ${s.distance.toFixed(3)} m`;
  }

  function frame(now) {
    if (playing) {
      if (lastFrame) accumulated += Math.min((now - lastFrame) / 1000, 0.25) * Number(rate.value);
      const steps = Math.floor(accumulated / core.DT);
      if (steps) {
        accumulated -= steps * core.DT;
        moveTo(index + steps);
        if (index === trial.samples.length - 1) { playing = false; play.textContent = 'Play controller'; }
      }
    }
    lastFrame = now;
    requestAnimationFrame(frame);
  }
  kind.addEventListener('change', configure);
  for (const id of poseInputs) $(id).addEventListener('change', configure);
  $('upstream-run').addEventListener('click', configure);
  play.addEventListener('click', () => {
    if (index === trial.samples.length - 1) moveTo(0);
    playing = !playing; play.textContent = playing ? 'Pause' : 'Play controller';
  });
  $('upstream-step').addEventListener('click', () => { playing = false; play.textContent = 'Play controller'; moveTo(index + 1); });
  $('upstream-reset').addEventListener('click', () => { playing = false; play.textContent = 'Play controller'; moveTo(0); });
  scrub.addEventListener('input', () => { playing = false; play.textContent = 'Play controller'; moveTo(Number(scrub.value)); });
  window.addEventListener('resize', render);
  configure();
  requestAnimationFrame(frame);
})();
