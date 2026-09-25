import assert from 'node:assert/strict';
import '../src/core.js';
import '../src/reeds-shepp.js';
import '../src/vehicle.js';

const C=globalThis.CarbotCore, V=globalThis.CarbotVehicle, RS=globalThis.CarbotRS;
const c={...C.DEFAULTS},g=C.geometry(c);
const close=(a,b,eps=1e-9)=>assert.ok(Math.abs(a-b)<eps,`${a} != ${b}`);
close(g.l,.275);close(g.w,.185);close(g.wb,.21);close(g.track,.160);
close(g.r,.4);close(g.wb/Math.tan(g.maxSteer),.4);
const origin={x:0,y:0,a:0};
const quarter=C.bicycle(origin,Math.PI*.4/2,1/.4);
close(quarter.x,.4);close(quarter.y,.4);close(quarter.a,Math.PI/2);
const back=C.bicycle(quarter,-Math.PI*.4/2,1/.4);
close(back.x,0);close(back.y,0);close(back.a,0);
console.log('PASS measured envelope, derived wheel spacing, signed bicycle geometry');

let candidateCount=0;
for(const goal of [{x:.8,y:.2,a:.5},{x:-.3,y:.4,a:Math.PI/2},{x:.1,y:-.35,a:-1.2},{x:-.7,y:0,a:0}]){
 const options=RS.candidates(origin,goal,.4);
 assert.ok(options.length>0,'At least one analytic connection');
 for(const q of options){
  candidateCount++;
  const end=q.path.at(-1);
  close(end.x,goal.x,1e-5);close(end.y,goal.y,1e-5);close(C.wrap(end.a-goal.a),0,1e-5);
  assert.ok(q.path.every(p=>Math.abs(p.k)<=2.5+1e-9));
  for(let i=1;i<q.path.length;i++)assert.ok(Math.hypot(q.path[i].x-q.path[i-1].x,q.path[i].y-q.path[i-1].y)<=.00601);
 }
}
console.log(`PASS ${candidateCount} independently checked Reeds–Shepp endpoints and sample spacing`);

let fresh={speed:.12,steer:.1,source:'ROAD',stamp:1};
assert.equal(V.arbitrate(fresh,1.19,null,c).winner,'ROAD');
assert.equal(V.arbitrate(fresh,1.201,null,c).winner,'WATCHDOG');
assert.equal(V.arbitrate(fresh,1.01,'Emergency stop',c).winner,'SAFETY STOP');
assert.equal(V.arbitrate(null,1,null,c).speed,0);
const parts=V.splitGears([{...origin,dir:1},{x:.1,y:0,a:0,dir:1},{x:.1,y:0,a:0,dir:-1},{x:0,y:0,a:0,dir:-1}]);
assert.equal(parts.length,2);assert.ok(parts[1].every(p=>p.dir===-1));
console.log('PASS safety veto, request expiry, and continuous gear cusp');

const estimator=new V.Estimator(origin,c);
estimator.update({stamp:1,encoder:.1,yaw:0,uwb:null},1,1);
close(estimator.pose.x,.1);
const frozenX=estimator.pose.x;
estimator.update({stamp:1,encoder:999,yaw:1,uwb:{x:999,y:999}},2,1);
close(estimator.pose.x,frozenX);
const mem=new V.LocalMemory();
mem.cells.set('0,0',{x:0,y:0,kind:1,stamp:1});
assert.ok(mem.query(0,0,20));assert.equal(mem.query(0,0,27),null);
console.log('PASS estimator consumes packets, rejects repeated packet, memory expires');

const course=new C.Course(c),mission=C.buildMission(course,c);
assert.equal(mission.routes.length,2);
for(const route of mission.routes){
 assert.ok(route.length>100);
 for(const p of route){assert.ok(course.bodyClear(p,c),`Nominal path outside road at ${p.x},${p.y}`);assert.equal(p.dir,1,'Road travel is forward');}
}
assert.ok(course.parked(course.parkGoal()));
const r=C.parkingPlan(mission.handoff,course.parkGoal(),course,c);
assert.ok(r.path.length>0,'Parking connection exists at default radius');
assert.ok(r.path.some(p=>p.dir===-1),'Parking uses reverse');
assert.ok(r.path.every(p=>course.bodyClear(p,c)));
console.log(`PASS directed mission planning (${mission.routes.map(p=>p.length).join('/')} samples), legal nominal parking path`);
console.log('All mathematical/control checks passed. These do not certify the physical car.');
