import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const script = fs.readFileSync(new URL('../upstream-parking-core.js', import.meta.url), 'utf8');
const sandbox = {window: {}};
vm.runInNewContext(script, sandbox);
const core = sandbox.window.UpstreamParkingCore;

assert.equal(core.DEFAULTS.parallel_forward_dist, 0.30);
assert.equal(core.DEFAULTS.parallel_reverse_dist, 0.35);
assert.equal(core.DEFAULTS.perp_turn_angle, 1.57);
assert.equal(core.DEFAULTS.park_wait_time, 3.0);

const parallel = core.simulate('parallel');
assert.equal(parallel.sourceBlob, '5ebef7aaaa281d04fb2e5bb888b9fc6e14256edf');
assert.equal(parallel.complete, true);
assert.equal(parallel.parked?.fits, false);
assert.equal(parallel.startsInside, false);
assert.deepEqual(Array.from(parallel.transitions, x => x.phase), [
  'PARALLEL_FORWARD', 'PARALLEL_STEER_REVERSE', 'PARALLEL_STRAIGHTEN',
  'PARALLEL_WAIT', 'PARALLEL_EXIT', 'IDLE',
]);
assert.ok(parallel.samples.some(x => x.linear === -0.12 && x.angular === -0.6));
assert.ok(parallel.samples.some(x => x.linear === 0.075 && x.angular === 0.3));
assert.ok(Math.abs(parallel.transitions[4].time - parallel.transitions[3].time - 3.0) < 0.051);

const perpendicular = core.simulate('perpendicular');
assert.equal(perpendicular.complete, true);
assert.equal(perpendicular.parked?.fits, false);
assert.deepEqual(Array.from(perpendicular.transitions, x => x.phase), [
  'PERP_TURN_IN', 'PERP_FORWARD', 'PERP_WAIT', 'PERP_REVERSE_OUT', 'IDLE',
]);
const turn = perpendicular.samples.filter(x => x.time <= perpendicular.transitions[1].time);
assert.ok(turn.every(x => x.x === perpendicular.samples[0].x &&
  x.y === perpendicular.samples[0].y && x.yaw === perpendicular.samples[0].yaw));
assert.ok(turn.some(x => x.linear === 0 && x.angular === 0.5));
assert.ok(Math.abs(perpendicular.transitions[1].time - 1.57 / 0.5) < 0.051);
assert.ok(perpendicular.samples.some(x => x.linear === -0.12 && x.angular === 0));

// Initial alignment matters: the harness reports containment, not a universal pass.
const aligned = core.simulate('parallel', {start: {x: 0.05, y: 0.20, yawDeg: 0}});
assert.equal(aligned.parked?.fits, true);
assert.equal(aligned.startsInside, true);

const elements = new Map();
const context2d = new Proxy({}, {get: (target, key) => target[key] ?? (() => {})});
for (const id of ['upstream-parking', 'upstream-canvas', 'upstream-kind',
  'upstream-x', 'upstream-y', 'upstream-heading', 'upstream-wheel',
  'upstream-depth',
  'upstream-scrub', 'upstream-play', 'upstream-rate', 'upstream-status',
  'upstream-result', 'upstream-run', 'upstream-step', 'upstream-reset']) {
  const initial = {'upstream-kind': 'parallel', 'upstream-x': '0.05',
    'upstream-y': '-0.18', 'upstream-heading': '0', 'upstream-wheel': '50',
    'upstream-depth': '0.40', 'upstream-rate': '1'};
  elements.set(id, {value: initial[id] ?? '0', clientWidth: 800, clientHeight: 480,
    listeners: {}, addEventListener(event, callback) { this.listeners[event] = callback; },
    getContext: () => context2d});
}
sandbox.window.devicePixelRatio = 1;
sandbox.window.addEventListener = () => {};
sandbox.document = {getElementById: id => elements.get(id)};
sandbox.requestAnimationFrame = () => {};
vm.runInNewContext(fs.readFileSync(new URL('../upstream-parking-ui.js', import.meta.url), 'utf8'), sandbox);
assert.match(elements.get('upstream-result').textContent, /BODY OUTSIDE BAY/);
elements.get('upstream-kind').value = 'perpendicular';
elements.get('upstream-kind').listeners.change();
assert.match(elements.get('upstream-result').textContent, /Turn-in heading change: 0\.0°/);
elements.get('upstream-step').listeners.click();
assert.match(elements.get('upstream-status').textContent, /0\.05 s.*PERP_TURN_IN/);
console.log('PASS upstream parking phases, Ackermann zero-speed turn, containment and UI controls');
