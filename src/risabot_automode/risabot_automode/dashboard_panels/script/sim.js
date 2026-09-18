let simLoaded = false;
function toggleSim() {
  const panel = document.getElementById('simPanel');
  const frame = document.getElementById('simFrame');
  const btn = document.getElementById('simBtn');
  const show = panel.style.display === 'none';
  panel.style.display = show ? 'block' : 'none';
  btn.textContent = show ? '📊 Disable V4 Views' : '📊 Enable V4 Views';
  if (show && !simLoaded) {
    simLoaded = true;
    frame.src = '/sim/index.html';
    frame.style.display = 'block';
    fetch('/api/replay/list').then(r => r.json()).then(data => {
      const sel = document.getElementById('replaySel');
      (data.replays || []).forEach(name => {
        const opt = document.createElement('option');
        opt.value = name;
        opt.textContent = name;
        sel.appendChild(opt);
      });
    }).catch(() => {});
  }
  if (!show) {
    frame.src = '';
    frame.style.display = 'none';
  }
}

function onReplayChange() {
  const name = document.getElementById('replaySel').value;
  const meta = document.getElementById('replayMeta');
  if (!name) {
    meta.textContent = 'No replay selected';
    return;
  }
  meta.textContent = 'Loading…';
  fetch('/api/replay/get?name=' + encodeURIComponent(name) + '&meta=1').then(r => r.json()).then(data => {
    if (data.ok && data.meta) {
      const m = data.meta;
      meta.textContent = m.frames + ' frames, ' + m.duration_s + 's' +
        (m.has_odom ? ', odom ✓' : ', no odom') +
        (m.corridor_frames ? ', corridor ' + m.corridor_frames : ', no corridor');
    } else {
      meta.textContent = 'Error: ' + (data.error || 'unknown');
    }
  }).catch(() => { meta.textContent = 'Load failed'; });
}

function openSimReplay() {
  const name = document.getElementById('replaySel').value;
  window.open('/sim/index.html' + (name ? '?replay=' + encodeURIComponent(name) : ''), '_blank');
}

