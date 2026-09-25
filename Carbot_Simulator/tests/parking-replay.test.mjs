import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const elements = new Map();
const context2d = new Proxy({}, {get: (target, key) => target[key] ?? (() => {})});
for (const id of ['r5-parking', 'parking-canvas', 'parking-recording', 'parking-play',
  'parking-scrub', 'parking-rate', 'parking-readout', 'parking-source', 'parking-legend',
  'parking-step', 'parking-reset']) {
  elements.set(id, {
    value: id === 'parking-recording' ? 'parallel' : id === 'parking-rate' ? '1' : '0',
    clientWidth: 800,
    clientHeight: 480,
    listeners: {},
    addEventListener(event, callback) { this.listeners[event] = callback; },
    getContext: () => context2d,
  });
}
const sandbox = {
  window: {devicePixelRatio: 1, addEventListener() {}},
  document: {getElementById: id => elements.get(id)},
  requestAnimationFrame() {},
};
vm.runInNewContext(fs.readFileSync(new URL('../parking-recordings.js', import.meta.url), 'utf8'), sandbox);
const data = sandbox.window.R5ParkingRecordings;
assert.equal(data.parallel.sample_count, 2048);
assert.equal(data.perpendicular.sample_count, 737);
for (const record of Object.values(data)) {
  assert.equal(record.runs.reduce((n, run) => n + run[2], 0), record.sample_count);
  assert.equal(record.duration_sec, record.sample_count / 20);
}
vm.runInNewContext(fs.readFileSync(new URL('../parking-replay.js', import.meta.url), 'utf8'), sandbox);
assert.match(elements.get('parking-readout').textContent, /102\.40 s.*motor PWM \+0.*sample 1\/2048/);
assert.match(elements.get('parking-source').textContent, /47dd70a048443d9d4137e1df26e69f667d0c73a8ba37a0a600f0ce041fac0557/);
elements.get('parking-recording').value = 'perpendicular';
elements.get('parking-recording').listeners.change();
assert.match(elements.get('parking-readout').textContent, /36\.85 s.*sample 1\/737/);
elements.get('parking-step').listeners.click();
assert.match(elements.get('parking-readout').textContent, /0\.05 \/ 36\.85 s.*sample 2\/737/);
console.log('PASS R5 parallel/perpendicular offline replay data and controls');
