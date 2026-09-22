import assert from 'node:assert/strict';
import '../src/core.js';import '../src/vehicle.js';import '../src/guidance.js';
const C=CarbotCore,V=CarbotVehicle,c={...C.DEFAULTS},origin={x:0,y:0,a:0};
const low=new V.Estimator(origin,{...c,uwbNoiseCm:1.2}),high=new V.Estimator(origin,{...c,uwbNoiseCm:20});
for(let i=1;i<=600;i++){
 const base={stamp:i,encoder:.08,yaw:0,uwb:null};
 low.update({...base,uwb:i%12===0?{x:.08*i/60,y:.005}:null},i/60,1/60);
 high.update({...base,uwb:i%12===0?{x:.08*i/60+.18,y:.2}:null},i/60,1/60);
 assert.deepEqual(high.pose,low.pose,'UWB must not change local steering pose');
}
assert.ok(high.uwbGain<low.uwbGain);assert.ok(high.sigma<.035);assert.ok(high.globalPose.y>low.globalPose.y);
const before={...high.pose};high.update({stamp:601,encoder:0,yaw:0,uwb:{x:100,y:100}},10.1,1/60);assert.equal(high.lastAccepted,false);assert.deepEqual(high.pose,before);
console.log('PASS high-noise UWB isolated from local pose; covariance weighting; extreme outlier rejection');
// Synthetic independent camera observation of a straight 30 cm road, with a
// deliberately incorrect 2 cm map alignment. Vision must correct lateral drift.
const n=70,res=.01,per={n,kind:new Uint8Array(n*n),grown:new Uint8Array(n*n),localPoint:i=>({x:Math.floor(i/n)*res-.15,y:i%n*res-.345})};
for(let i=0;i<n*n;i++){const p=per.localPoint(i);per.kind[i]=Math.abs(p.y)<.15?1:2;per.grown[i]=per.kind[i]===1?1:0}
const est=new V.Estimator(origin,c);est.transform.y=.02;est.refresh();const road={clearance:(x,y)=>.15-Math.abs(y)};
for(let i=0;i<80;i++)est.visualUpdate(per,road,i/8);
assert.ok(Math.abs(est.pose.y)<.006,`Visual correction residual ${est.pose.y}`);assert.ok(est.visualMatches>=8);
assert.ok(est.visualRank<.25,'Straight road must not claim strong along-road observability');
console.log('PASS image boundary correction and straight-corridor observability distinction');
const mem=new V.LocalMemory();mem.cells.set('0,0',{x:0,y:0,kind:1,stamp:0,distance:0});
const sim={estimate:{transform:{x:0,y:0},odom:origin,distance:0},perception:{stamp:-100,support:()=>0},memory:mem,t:1};
assert.equal(CarbotGuidance.evidence(sim,origin)?.kind,1);sim.estimate.distance=4;assert.equal(CarbotGuidance.evidence(sim,origin),null,'Travel uncertainty expires planning evidence');sim.estimate.distance=0;sim.t=4;assert.equal(CarbotGuidance.evidence(sim,origin),null,'Planning memory expires earlier than display memory');
console.log('PASS remembered evidence limited by age and accumulated motion');
const path=Array.from({length:101},(_,i)=>({x:i*.01,y:0,a:0,dir:1}));
const laneSim={c,path,ctrl:{index:0},t:0,estimate:{pose:origin,odom:origin,transform:{x:0,y:0},distance:0,globalOffset:{x:0,y:0},globalSigma:.02,visualRank:0},cameraOdom:origin,perception:{stamp:0,support:p=>Math.abs(p.y-.02)<.15?1:Math.abs(p.y-.02)<.22?-1:0},memory:new V.LocalMemory(),course:{clearance:(x,y)=>.15-Math.abs(y),round:{x:.6,y:0},laneChange:{x0:10,x1:11,y0:10,y1:11}}};
const guidance=CarbotGuidance.corridor(laneSim);assert.ok(guidance.observed>=3);assert.ok(guidance.offset>.005,'Measured shifted lane must move the steering reference');laneSim.guidance=guidance;
assert.equal(CarbotGuidance.branchCheck(laneSim).hold,false);laneSim.estimate.globalOffset.x=1;assert.equal(CarbotGuidance.branchCheck(laneSim).hold,true,'Conflicting course location at a branch must hold');laneSim.estimate.visualRank=.5;assert.equal(CarbotGuidance.branchCheck(laneSim).hold,false,'Distinctive visual boundary geometry can support route identity despite UWB disagreement');
console.log('PASS camera mask changes steering reference; branch conflict hold and visual agreement');
