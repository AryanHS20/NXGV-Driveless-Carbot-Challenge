import assert from 'node:assert/strict';
import {dashboardOdom, alignAtStart, coursePose} from '../src/live-car.js';

const packet = {odom_x: 10, odom_y: 20, odom_yaw: 0, speed: 0.12, freshness_sec: {odom: 0.08}};
const odom = dashboardOdom(packet);
assert.deepEqual(odom, {x: 10, y: 20, a: 0, speed: 0.12, age: 0.08});
assert.equal(dashboardOdom({...packet, freshness_sec: {odom: 1.2}}), null);
assert.equal(dashboardOdom({...packet, freshness_sec: {odom_sim: 0.01}}), null);
assert.equal(dashboardOdom({...packet, odom_yaw: NaN}), null);

const start = {x: 7.18, y: 1.3025, a: Math.PI};
const alignment = alignAtStart(odom, start);
const initial = coursePose(odom, alignment);
assert.ok(Math.abs(initial.x - start.x) < 1e-9 && Math.abs(initial.y - start.y) < 1e-9);
assert.ok(Math.abs(Math.abs(initial.a) - Math.PI) < 1e-9);
const westward = coursePose({...odom, x: 11}, alignment);
assert.ok(Math.abs(westward.x - 6.18) < 1e-9);
assert.ok(Math.abs(westward.y - start.y) < 1e-9);
console.log('PASS live hardware-odom freshness gate and start-frame alignment');
