/* Offline display of the recorded R5 motor PWM and steering sequence.
   The drawn path is illustrative: recordings contain no measured pose. */
(() => {
  'use strict';
  const records = window.R5ParkingRecordings;
  const root = document.getElementById('r5-parking');
  if (!root || !records) return;

  const $ = id => document.getElementById(id);
  const canvas = $('parking-canvas');
  const ctx = canvas.getContext('2d');
  const selector = $('parking-recording');
  const playButton = $('parking-play');
  const scrub = $('parking-scrub');
  const rate = $('parking-rate');
  const readout = $('parking-readout');
  const provenance = $('parking-source');
  const legend = $('parking-legend');
  const DT = 0.05;
  const WHEELBASE = 0.21;
  const ASSUMED_SPEED_AT_100 = 0.08; // metres/second; only for the sketch
  const MAX_WHEEL_ANGLE = 50 * Math.PI / 180; // provisional, not measured

  let recording;
  let samples;
  let poses;
  let index = 0;
  let playing = false;
  let lastFrame = 0;
  let accumulated = 0;

  function setRecording(kind) {
    recording = records[kind];
    samples = recording.runs.flatMap(([pwm, angle, count]) =>
      Array.from({length: count}, () => ({pwm, angle})));
    if (samples.length !== recording.sample_count) throw new Error('Parking recording sample count mismatch');
    poses = [{x: 0, y: 0, heading: 0}];
    for (const {pwm, angle} of samples) {
      const prev = poses[poses.length - 1];
      const steeringFraction = angle < 80 ? (angle - 80) / 50 : (angle - 80) / 70;
      const wheelAngle = steeringFraction * MAX_WHEEL_ANGLE;
      const distance = pwm / 100 * ASSUMED_SPEED_AT_100 * DT;
      const heading = prev.heading + distance * Math.tan(wheelAngle) / WHEELBASE;
      poses.push({
        x: prev.x + distance * Math.cos((prev.heading + heading) / 2),
        y: prev.y + distance * Math.sin((prev.heading + heading) / 2),
        heading,
      });
    }
    index = 0;
    playing = false;
    accumulated = 0;
    playButton.textContent = 'Play recording';
    scrub.max = samples.length - 1;
    provenance.textContent = `${recording.name} · ${recording.sample_count} samples at 20 Hz · SHA-256 ${recording.sha256}`;
    render();
  }

  function moveTo(value) {
    index = Math.max(0, Math.min(samples.length - 1, Math.floor(value)));
    scrub.value = String(index);
    render();
  }

  function render() {
    const width = Math.max(300, Math.round(canvas.clientWidth));
    const height = Math.max(280, Math.round(canvas.clientHeight));
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0d1a26';
    ctx.fillRect(0, 0, width, height);

    const plotH = Math.max(140, height * 0.56);
    const xs = poses.map(p => p.x);
    const ys = poses.map(p => p.y);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const span = Math.max(maxX - minX, maxY - minY, 0.12);
    const scale = Math.min((width - 86) / span, (plotH - 70) / span);
    const cx = width / 2 - (minX + maxX) * scale / 2;
    const cy = plotH / 2 + (minY + maxY) * scale / 2;
    const screen = p => [cx + p.x * scale, cy - p.y * scale];

    ctx.strokeStyle = '#1e3243';
    ctx.lineWidth = 1;
    for (let x = 20; x < width; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, plotH); ctx.stroke(); }
    for (let y = 20; y < plotH; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
    ctx.font = '12px system-ui';
    ctx.fillStyle = '#a6bbca';
    ctx.fillText('Illustrative path sketch — distance and start pose not measured', 15, 22);
    ctx.fillStyle = '#4ce5a0';
    ctx.fillText('● forward', 15, plotH - 14);
    ctx.fillStyle = '#ffbc72';
    ctx.fillText('● reverse', 105, plotH - 14);

    for (let i = 1; i <= index; i++) {
      if (samples[i - 1].pwm === 0) continue;
      const a = screen(poses[i - 1]), b = screen(poses[i]);
      ctx.strokeStyle = samples[i - 1].pwm > 0 ? '#4ce5a0' : '#ffbc72';
      ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    }
    const pose = poses[index];
    const [carX, carY] = screen(pose);
    ctx.save();
    ctx.translate(carX, carY);
    ctx.rotate(-pose.heading);
    ctx.fillStyle = '#4ed9ed';
    ctx.strokeStyle = '#d9f8ff';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.roundRect(-18, -10, 36, 20, 4); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#071720';
    ctx.beginPath(); ctx.moveTo(16, 0); ctx.lineTo(7, -6); ctx.lineTo(7, 6); ctx.closePath(); ctx.fill();
    ctx.restore();

    const chartTop = plotH + 12;
    const chartBottom = height - 30;
    const mid = (chartTop + chartBottom) / 2;
    ctx.fillStyle = '#a6bbca';
    ctx.fillText('Actual recorded commands (PWM / steering)', 15, chartTop + 5);
    ctx.strokeStyle = '#40566a';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(15, mid); ctx.lineTo(width - 15, mid); ctx.stroke();
    const chartX = i => 18 + i / (samples.length - 1) * (width - 36);
    function line(value, color) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < samples.length; i += Math.max(1, Math.floor(samples.length / width))) {
        const y = mid - value(samples[i]) * (chartBottom - chartTop) * 0.36;
        if (i === 0) ctx.moveTo(chartX(i), y); else ctx.lineTo(chartX(i), y);
      }
      ctx.stroke();
    }
    line(s => s.pwm / 100, '#4ce5a0');
    line(s => (s.angle - 80) / 70, '#ffdc6e');
    const marker = chartX(index);
    ctx.strokeStyle = '#ffffff';
    ctx.beginPath(); ctx.moveTo(marker, chartTop + 12); ctx.lineTo(marker, chartBottom); ctx.stroke();
    ctx.fillStyle = '#a6bbca';
    ctx.fillText('0 s', 18, height - 7);
    ctx.fillText(`${recording.duration_sec.toFixed(2)} s`, width - 70, height - 7);

    const command = samples[index];
    const phase = command.pwm > 0 ? 'FORWARD' : command.pwm < 0 ? 'REVERSE' : 'STOPPED';
    readout.textContent = `${(index * DT).toFixed(2)} / ${recording.duration_sec.toFixed(2)} s  ·  ${phase}  ·  motor PWM ${command.pwm >= 0 ? '+' : ''}${command.pwm}  ·  servo ${command.angle}  ·  sample ${index + 1}/${samples.length}`;
    legend.textContent = 'Green: recorded motor PWM · Yellow: recorded steering deviation from centre 80 · White: replay position';
  }

  function frame(now) {
    if (playing) {
      if (lastFrame) accumulated += Math.min((now - lastFrame) / 1000, 0.25) * Number(rate.value);
      const steps = Math.floor(accumulated / DT);
      if (steps) {
        accumulated -= steps * DT;
        moveTo(index + steps);
        if (index === samples.length - 1) {
          playing = false;
          playButton.textContent = 'Play recording';
        }
      }
    }
    lastFrame = now;
    requestAnimationFrame(frame);
  }

  selector.addEventListener('change', () => setRecording(selector.value));
  playButton.addEventListener('click', () => {
    if (index === samples.length - 1) moveTo(0);
    playing = !playing;
    playButton.textContent = playing ? 'Pause' : 'Play recording';
  });
  $('parking-step').addEventListener('click', () => { playing = false; playButton.textContent = 'Play recording'; moveTo(index + 1); });
  $('parking-reset').addEventListener('click', () => { playing = false; playButton.textContent = 'Play recording'; moveTo(0); });
  scrub.addEventListener('input', () => { playing = false; playButton.textContent = 'Play recording'; moveTo(Number(scrub.value)); });
  window.addEventListener('resize', render);
  setRecording(selector.value);
  requestAnimationFrame(frame);
})();
