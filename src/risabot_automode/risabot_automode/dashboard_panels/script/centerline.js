function updateCenterline(d) {
  const canvas = document.getElementById('centerlineCanvas');
  const badge = document.getElementById('centerlineBadge');
  const meta = document.getElementById('centerlineMeta');
  if (!canvas || !badge || !meta) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, bottom = H - 29, scale = 188;
  const project = (forward, left) => [cx - left * scale, bottom - forward * scale];
  const finite = n => typeof n === 'number' && Number.isFinite(n);
  const v4 = d && d.v4_status || {};
  const road = v4.road || {};
  const trajectory = v4.trajectory || {};
  const raw = road.corridor && road.corridor.primary || [];
  const rows = Array.isArray(raw) ? raw.filter(r => r && finite(r.forward_m) &&
    finite(r.left_m) && finite(r.width_m) && r.forward_m >= 0 &&
    r.forward_m <= 1.8 && r.width_m > 0).slice().sort((a,b) => a.forward_m - b.forward_m) : [];
  const roadAge = road.age_sec;
  const planAge = trajectory.age_sec;
  const fresh = finite(roadAge) && roadAge <= 0.35 &&
    finite(planAge) && planAge <= 0.35;
  const selected = fresh && trajectory.selected_diagnostic_only &&
    trajectory.selected_diagnostic_only.valid === true ?
    trajectory.selected_diagnostic_only : null;

  ctx.fillStyle = '#111923'; ctx.fillRect(0, 0, W, H);
  ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(170,185,205,.19)';
  ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = '#9aaabd';
  for (const m of [0, .5, 1, 1.5]) {
    const y = project(m, 0)[1];
    ctx.beginPath(); ctx.moveTo(30, y); ctx.lineTo(W-18, y); ctx.stroke();
    ctx.fillText(m.toFixed(1) + ' m', 35, y - 5);
  }
  const axis = project(1.8, 0);
  ctx.setLineDash([4, 7]); ctx.strokeStyle = 'rgba(170,185,205,.30)';
  ctx.beginPath(); ctx.moveTo(cx, bottom); ctx.lineTo(axis[0], axis[1]); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#c1cbd8'; ctx.fillText('LEFT', 60, H-12);
  ctx.fillText('RIGHT', W-88, H-12);

  function drawRun(points, color, width, dash) {
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.setLineDash(dash || []);
    let previous = null;
    for (const point of points) {
      if (!point) { previous = null; continue; }
      const pixel = project(point[0], point[1]);
      if (previous && point[0] - previous[0] <= .09) {
        const a = project(previous[0], previous[1]);
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(pixel[0], pixel[1]); ctx.stroke();
      }
      previous = point;
    }
    ctx.setLineDash([]);
  }
  const left = [], right = [], measured = [], estimated = [];
  for (const r of rows) {
    const l = r.left_boundary_observed === true;
    const q = r.right_boundary_observed === true;
    left.push(l ? [r.forward_m, r.left_m + r.width_m/2] : null);
    right.push(q ? [r.forward_m, r.left_m - r.width_m/2] : null);
    measured.push(l && q ? [r.forward_m, r.left_m] : null);
    const estimate = l && !q ? r.left_m + r.width_m/2 - .16 :
      q && !l ? r.left_m - r.width_m/2 + .16 : null;
    estimated.push(estimate !== null ? [r.forward_m, estimate] : null);
  }
  drawRun(left, '#f4f4f0', 2, []);
  drawRun(right, '#f4f4f0', 2, []);
  drawRun(estimated, '#f5b85b', 2, [5, 5]);
  drawRun(measured, '#2de2d0', 3, []);
  for (const point of measured) {
    if (!point) continue;
    const p = project(point[0], point[1]);
    ctx.fillStyle = '#2de2d0'; ctx.beginPath(); ctx.arc(p[0], p[1], 2.3, 0, 2*Math.PI); ctx.fill();
  }

  // Rear-axle marker and currently issued steering, not saved route playback.
  ctx.fillStyle = '#65a7ff';
  ctx.beginPath(); ctx.moveTo(cx, bottom-15); ctx.lineTo(cx-11, bottom+4);
  ctx.lineTo(cx+11, bottom+4); ctx.closePath(); ctx.fill();
  const moving = d && d.auto_mode === true && finite(d.cmd_lin_x) && d.cmd_lin_x > .001;
  if (moving && finite(d.cmd_ang_z)) {
    const wheelAngle = -d.cmd_ang_z * (50 * Math.PI / 180);
    const curvature = Math.tan(wheelAngle) / .21;
    let x = 0, y = 0, yaw = 0;
    ctx.strokeStyle = '#a78bfa'; ctx.lineWidth = 2.5; ctx.setLineDash([7, 4]);
    ctx.beginPath(); ctx.moveTo(cx, bottom);
    for (let distance = .015; distance <= .66; distance += .015) {
      const mid = yaw + .0075 * curvature;
      x += .015 * Math.cos(mid); y += .015 * Math.sin(mid);
      yaw += .015 * curvature;
      const p = project(x, y); ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke(); ctx.setLineDash([]);
  }
  if (selected && finite(selected.target_forward_m) && finite(selected.target_left_m)) {
    const p = project(selected.target_forward_m, selected.target_left_m);
    ctx.strokeStyle = '#ff6486'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(p[0], p[1], 9, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p[0]-13, p[1]); ctx.lineTo(p[0]+13, p[1]);
    ctx.moveTo(p[0], p[1]-13); ctx.lineTo(p[0], p[1]+13); ctx.stroke();
  }
  badge.textContent = !fresh ? 'STALE' : selected ? (moving ? 'FOLLOWING' : 'TARGET READY') : 'NO PATH';
  badge.style.color = !fresh || !selected ? '#f38ba8' : moving ? '#40a02b' : '#df8e1d';
  const near = measured.filter(Boolean).length;
  const targetText = selected ? 'target ' + selected.target_forward_m.toFixed(2) + ' m ahead, ' +
    (selected.target_left_m >= 0 ? 'left ' : 'right ') +
    Math.abs(selected.target_left_m).toFixed(3) + ' m' :
    (trajectory.last_error || 'no selected target');
  meta.textContent = (d.auto_mode ? 'AUTO' : 'MANUAL') + ' · ' +
    (trajectory.lane_controller || 'controller unknown') + ' · ' +
    near + ' measured centre points · ' + targetText;
}
