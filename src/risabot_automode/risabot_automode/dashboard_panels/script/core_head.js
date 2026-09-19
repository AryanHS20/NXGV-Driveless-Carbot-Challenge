<script>
let eventLog = [];
const PRIORITY_ORDER = ['LANE_FOLLOW', 'TUNNEL', 'OBSTRUCTION', 'BOOM_GATE', 'TRAFFIC_LIGHT', 'MANUAL'];
let lastState = '';
let lastLap = 0;
let lapStartTime = Date.now();
const dashStartTime = Date.now();
let lastStopReason = '';

// Session uptime timer
setInterval(() => {
  const elapsed = Math.floor((Date.now() - dashStartTime) / 1000);
  const h = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const m = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const s = String(elapsed % 60).padStart(2, '0');
  document.getElementById('uptimeText').textContent = h + ':' + m + ':' + s;
  // Lap timer
  const lapElapsed = Math.floor((Date.now() - lapStartTime) / 1000);
  const lm = String(Math.floor(lapElapsed / 60)).padStart(2, '0');
  const ls = String(lapElapsed % 60).padStart(2, '0');
  document.getElementById('lapTimer').textContent = lm + ':' + ls;
}, 1000);

