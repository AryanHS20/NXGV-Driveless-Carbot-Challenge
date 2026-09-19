const DASH_PAGES = {
  drive: ['Drive', 'Competition controls and the signals that can stop the car.'],
  perception: ['Perception', 'Camera, lane, LiDAR, and detector outputs in one diagnostic view.'],
  v4: ['V4 Readiness', 'Live status and validation blockers for every V4 stage.'],
  calibration: ['Calibration', 'Sensor alignment, attitude, odometry, and V4 calibration gates.'],
  runs: ['Runs & Replays', 'Record trials, inspect events, and replay V4 data.'],
  system: ['System', 'Sensor freshness, health, controller input, and event history.']
};

function showPage(name, updateHash=true) {
  if (!DASH_PAGES[name]) name = 'drive';
  document.querySelectorAll('[data-pages]').forEach(el => {
    const pages = (el.dataset.pages || '').split(',');
    el.classList.toggle('page-hidden', !pages.includes(name));
  });
  document.querySelectorAll('.nav-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.page === name);
  });
  document.getElementById('pageTitle').textContent = DASH_PAGES[name][0];
  document.getElementById('pageDescription').textContent = DASH_PAGES[name][1];
  document.title = DASH_PAGES[name][0] + ' · RISA-Bot';
  if (updateHash && location.hash !== '#' + name) history.pushState(null, '', '#' + name);

  // Streams are opt-in on every page. Leaving their visible workspace releases
  // the HTTP connection/animation so hidden views cannot consume stale frames.
  const cameraPages = ['drive', 'perception', 'calibration'];
  const camBtn = document.getElementById('camBtn');
  if (!cameraPages.includes(name) && camBtn && camBtn.classList.contains('active')) toggleCam();
  const simPanel = document.getElementById('simPanel');
  if (!['v4', 'runs'].includes(name) && simPanel && simPanel.style.display !== 'none') toggleSim();
  if (!['drive', 'v4'].includes(name) && typeof drivevizOn !== 'undefined' && drivevizOn) toggleDriveviz();
}

window.addEventListener('popstate', () => showPage(location.hash.slice(1), false));
showPage(location.hash.slice(1) || 'drive', false);

function updateAuthority(d) {
  const mode = d.auto_mode ? 'AUTO' : 'MANUAL';
  const source = String(d.cmd_safety_autonomy_source || 'unknown').toUpperCase();
  const estop = Boolean(d.cmd_safety_estop);
  const lin = Number(d.cmd_lin_x || 0), ang = Number(d.cmd_ang_z || 0);
  const stopped = Math.abs(lin) < 0.001 && Math.abs(ang) < 0.001;
  const reason = estop ? 'Safety controller emergency stop' :
    (d.stop_reason || (stopped ? (d.auto_mode ? 'No motion command' : 'Manual mode') : 'Moving'));
  const values = {
    authorityMode: [mode, d.auto_mode ? 'ok' : 'warn'],
    authoritySource: [source, source === 'V4' ? 'ok' : (source === 'LEGACY' ? 'warn' : '')],
    authoritySafety: [estop ? 'E-STOP' : 'CLEAR', estop ? 'bad' : 'ok'],
    authorityCommand: [lin.toFixed(3) + ' / ' + ang.toFixed(3), stopped ? 'warn' : 'ok'],
    authorityReason: [reason, estop || d.stop_reason ? 'bad' : (stopped ? 'warn' : 'ok')]
  };
  Object.entries(values).forEach(([id, item]) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = item[0];
    el.className = 'authority-value ' + item[1];
    el.title = item[0];
  });
}

function updateV4Status(d) {
  const statuses = d.v4_status || {};
  let waiting = 0, blocked = 0, ready = 0, firstBlocker = '';
  document.querySelectorAll('[data-v4-stage]').forEach(card => {
    const status = statuses[card.dataset.v4Stage];
    const stateEl = card.querySelector('.v4-stage-state');
    const detailEl = card.querySelector('.v4-stage-detail');
    let label = 'WAITING', klass = 'v4-waiting', detail = 'No status received';
    if (!status) {
      waiting += 1;
    } else if (status.age_sec == null || status.age_sec > 3) {
      label = 'STALE'; klass = 'v4-stale'; detail = status.age_sec == null ? 'Age unavailable' : status.age_sec.toFixed(1) + ' s old';
      blocked += 1;
      if (!firstBlocker) firstBlocker = card.dataset.v4Stage + ': status is stale';
    } else {
      const blockers = Array.isArray(status.blockers) ? status.blockers : [];
      const error = status.last_error || '';
      if (status.enabled === false) blockers.unshift('node disabled');
      if (blockers.length || error) {
        label = 'BLOCKED'; klass = 'v4-blocked'; detail = blockers[0] || error;
        blocked += 1;
        if (!firstBlocker) firstBlocker = card.dataset.v4Stage + ': ' + detail;
      } else {
        label = status.motion_authority ? 'ARMED' : 'READY'; klass = 'v4-ready';
        detail = status.reason || status.source || ('Live · ' + status.age_sec.toFixed(1) + ' s');
        ready += 1;
      }
    }
    stateEl.textContent = label;
    stateEl.className = 'v4-stage-state ' + klass;
    detailEl.textContent = detail;
    detailEl.title = detail;
  });
  const summary = document.getElementById('v4Summary');
  if (summary) {
    summary.textContent = ready + ' READY · ' + blocked + ' BLOCKED · ' + waiting + ' WAITING';
    summary.className = 'v4-stage-state ' + (blocked ? 'v4-blocked' : (waiting ? 'v4-waiting' : 'v4-ready'));
  }
  const blocker = document.getElementById('v4Blocker');
  if (blocker) blocker.textContent = firstBlocker || (waiting ? 'Waiting for V4 status topics.' : 'All reported V4 stages are clear.');
}
