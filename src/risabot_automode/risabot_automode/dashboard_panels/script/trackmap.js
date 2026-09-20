let trackmapOn = false;
let trackmapTimer = 0;
let trackmapTrail = [];
let trackmapCenter = null;

function toggleTrackmap() {
  const panel = document.getElementById('trackmapPanel');
  const btn = document.getElementById('trackmapBtn');
  trackmapOn = !trackmapOn;
  panel.style.display = trackmapOn ? 'block' : 'none';
  btn.textContent = trackmapOn ? '🗺️ Disable Track Map' : '🗺️ Enable Track Map';
  if (trackmapOn) {
    trackmapTick();
    trackmapTimer = setInterval(trackmapTick, 500);
  } else {
    clearInterval(trackmapTimer);
  }
}

function resetTrackmapTrail() {
  trackmapTrail = [];
}

function resetTrackmapView() {
  trackmapCenter = null;
}

function trackmapTick() {
  fetch('/api/v4_telemetry').then(r => r.json()).then(doc => {
    drawTrackmap(doc);
  }).catch(() => {});
}

function trackmapProject(x, y, scale) {
  const canvas = document.getElementById('trackmapCanvas');
  const cx = trackmapCenter ? trackmapCenter.x : canvas.width / 2;
  const cy = trackmapCenter ? trackmapCenter.y : canvas.height / 2;
  return [cx + x * scale, cy - y * scale];
}

function drawTrackmap(doc) {
  const canvas = document.getElementById('trackmapCanvas');
  const meta = document.getElementById('trackmapMeta');
  const badge = document.getElementById('trackmapStale');
  if (!canvas || !trackmapOn) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const SCALE = 40;
  ctx.clearRect(0, 0, W, H);

  if (!doc || !doc.ok || !doc.telemetry) {
    badge.textContent = '● Waiting';
    badge.style.color = '#888';
    meta.textContent = 'No telemetry yet';
    return;
  }
  const t = doc.telemetry;
  const stale = t.stale || [];
  const isStale = stale.length > 0;
  badge.textContent = isStale ? ('● Stale: ' + stale.slice(0, 3).join(',')) : '● Live';
  badge.style.color = isStale ? '#f38ba8' : '#9bd69b';

  const local = t.local || null;
  const coarse = t.coarse || null;

  // Trail from local poses (odom frame); auto-reset on jumps.
  if (local) {
    const last = trackmapTrail[trackmapTrail.length - 1];
    if (!last) {
      trackmapTrail.push([local.x, local.y]);
    } else {
      const moved = Math.hypot(local.x - last[0], local.y - last[1]);
      if (moved > 0.5) {
        trackmapTrail = [[local.x, local.y]];
      } else if (moved > 0.02) {
        trackmapTrail.push([local.x, local.y]);
        if (trackmapTrail.length > 2000) trackmapTrail.shift();
      }
    }
  }
  if (trackmapCenter === null && trackmapTrail.length > 0) {
    trackmapCenter = {x: W / 2 - trackmapTrail[0][0] * SCALE,
                      y: H / 2 + trackmapTrail[0][1] * SCALE};
  }

  function toPx(x, y) {
    return trackmapProject(x, y, SCALE);
  }
  function robotToWorld(fwd, left) {
    if (!local || typeof local.yaw !== 'number') return null;
    const c = Math.cos(local.yaw), s = Math.sin(local.yaw);
    return [local.x + fwd * c - left * s, local.y + fwd * s + left * c];
  }

  // Trail.
  ctx.strokeStyle = isStale ? 'rgba(139,147,165,0.5)' : 'rgba(66,165,245,0.85)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  trackmapTrail.forEach(function(p, i) {
    const q = toPx(p[0], p[1]);
    if (i === 0) ctx.moveTo(q[0], q[1]);
    else ctx.lineTo(q[0], q[1]);
  });
  ctx.stroke();

  // Corridor (robot-relative, via fresh local pose).
  if (local && Array.isArray(t.corridor)) {
    ctx.strokeStyle = 'rgba(105,240,174,0.8)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    t.corridor.forEach(function(p) {
      const w = robotToWorld(p.forward_m, p.left_m);
      if (!w) return;
      const q = toPx(w[0], w[1]);
      if (!started) { ctx.moveTo(q[0], q[1]); started = true; }
      else ctx.lineTo(q[0], q[1]);
    });
    ctx.stroke();
  }

  // Selected trajectory endpoint + guide from local pose.
  if (local && t.trajectory && typeof t.trajectory.forward_m === 'number') {
    const w = robotToWorld(t.trajectory.forward_m, t.trajectory.left_m);
    if (w) {
      const a = toPx(local.x, local.y), b = toPx(w[0], w[1]);
      ctx.strokeStyle = 'rgba(0,229,255,0.9)';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#00e5ff';
      ctx.beginPath(); ctx.arc(b[0], b[1], 4, 0, Math.PI * 2); ctx.fill();
    }
  }

  // Obstacles (robot frame, via fresh local pose).
  if (local && Array.isArray(t.obstacles)) {
    t.obstacles.forEach(function(p) {
      const w = robotToWorld(p[0], p[1]);
      if (!w) return;
      const q = toPx(w[0], w[1]);
      const dist = Math.hypot(p[0], p[1]);
      ctx.fillStyle = dist < 0.5 ? '#ff5252' : (dist < 1.5 ? '#ffd740' : 'rgba(105,240,174,0.7)');
      ctx.beginPath(); ctx.arc(q[0], q[1], 2.5, 0, Math.PI * 2); ctx.fill();
    });
  }

  // Coarse pose (own frame): violet marker + sigma circle.
  if (coarse) {
    const q = toPx(coarse.x, coarse.y);
    if (typeof coarse.sigma_m === 'number' && coarse.sigma_m > 0) {
      ctx.strokeStyle = 'rgba(206,147,216,0.6)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(q[0], q[1], coarse.sigma_m * SCALE, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.fillStyle = '#ce93d8';
    ctx.beginPath(); ctx.arc(q[0], q[1], 4, 0, Math.PI * 2); ctx.fill();
  }

  // Local pose marker: blue dot + heading tick.
  if (local) {
    const q = toPx(local.x, local.y);
    ctx.fillStyle = '#42a5f5';
    ctx.beginPath(); ctx.arc(q[0], q[1], 5, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#42a5f5';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(q[0], q[1]);
    ctx.lineTo(q[0] + Math.cos(local.yaw) * 12, q[1] - Math.sin(local.yaw) * 12);
    ctx.stroke();
  }

  const age = (typeof doc.age_sec === 'number') ? doc.age_sec.toFixed(1) + 's' : '—';
  meta.textContent = 'frames: trail ' + trackmapTrail.length +
    ' · corridor ' + (Array.isArray(t.corridor) ? t.corridor.length : 0) +
    ' · obstacles ' + (Array.isArray(t.obstacles) ? t.obstacles.length : 0) +
    ' · age ' + age;
}
