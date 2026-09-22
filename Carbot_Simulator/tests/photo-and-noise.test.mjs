import assert from 'node:assert/strict';
import '../src/core.js';import '../src/reeds-shepp.js';import '../src/vehicle.js';import '../src/local-planner.js';
const C=CarbotCore,V=CarbotVehicle,c={...C.DEFAULTS},m=new C.Course(c);
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-8,`${a} != ${b}`);
near(m.laneChange.x1-m.laneChange.x0,.8);near(m.laneChange.y1-m.laneChange.y0,.8);
near(m.parallel.x1-m.parallel.x0,.47);near(m.parallel.y1-m.parallel.y0,.69);
near(m.parallelOuter.x1-m.parallelOuter.x0,.53);near(m.parallelOuter.y1-m.parallelOuter.y0,.75);
near(m.perpendicular.x1-m.perpendicular.x0,.45);near(m.perpendicular.y1-m.perpendicular.y0,.47);
near(m.perpendicular.x0-.03-m.perpendicularZone.x0,.17);near(m.perpendicularZone.x1-m.perpendicular.x1-.03,.24);
assert.ok(m.clearance(3.6,1.3)>0&&m.clearance(4.3,.8)>0,'Lane change is an open rectangle, not an S corridor');
const sensor=new V.Sensors({...c,noiseEnabled:false,uwbNoiseEnabled:true,uwbNoiseCm:20}),plant={pose:{x:0,y:0,a:0},speed:0,pitch:0,roll:0};let xs=[],ys=[];
for(let i=0;i<3000;i++){let p=sensor.sample(plant,i*.21,.21);xs.push(p.uwb.x);ys.push(p.uwb.y);near(p.yaw,0)}
const sd=v=>{let avg=v.reduce((a,b)=>a+b,0)/v.length;return Math.sqrt(v.reduce((a,b)=>a+(b-avg)**2,0)/v.length)};
assert.ok(Math.abs(sd(xs)-.2)<.012&&Math.abs(sd(ys)-.2)<.012);
sensor.c.uwbNoiseEnabled=false;let clean=sensor.sample(plant,700,.2);near(clean.uwb.x,0);near(clean.uwb.y,0);
const est=new V.Estimator(m.start,{...c,uwbNoiseCm:20,uwbNoiseEnabled:true});for(let i=1;i<10;i++)est.update({stamp:i,encoder:0,yaw:m.start.a,uwb:{x:m.start.x,y:m.start.y}},i,.2);assert.ok(est.sigma<.035,'UWB measurement noise must not inflate local steering uncertainty');assert.ok(est.uwbGain<.05,'Noisy UWB receives a small global-only gain');
console.log(`PASS supplied geometry; independent UWB toggle; sampled 20 cm sigma (${(100*sd(xs)).toFixed(1)}/${(100*sd(ys)).toFixed(1)} cm); noise-aware uncertainty`);
const mission=C.buildMission(m,c),mock={c,course:m,path:mission.routes[0],ctrl:{index:0},plant:{speed:0},steeringEstimate:0,estimate:{pose:m.start,odom:m.start},perception:{support:()=>1},memory:new V.LocalMemory(),t:0};
const pool=CarbotLocal.candidates(mock),valid=pool.filter(q=>q.valid);assert.equal(pool.length,9);assert.ok(valid.length);assert.equal(pool[0].cost,Math.min(...valid.map(q=>q.cost)));
for(const q of pool){near(q.points[0].x,m.start.x);near(q.points[0].y,m.start.y);for(let i=1;i<q.points.length;i++){assert.ok(Math.abs(q.points[i].k)<=1/C.geometry(c).r+1e-9);assert.ok(Math.hypot(q.points[i].x-q.points[i-1].x,q.points[i].y-q.points[i-1].y)<=.010001)}if(q.valid)assert.ok(q.points.every(p=>m.bodyClear(p,c)))}
console.log('PASS nine real rollouts from the estimated start, curvature bounds, nominal-body feasibility and lowest-cost feasible selection');
