(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const el = (tag, text, cls) => { const n=document.createElement(tag); if(text!=null)n.textContent=text; if(cls)n.className=cls; return n; };
  let catalog=null, token=null, planning=null, telemetry=null, online=false;
  let tabs=['cal-overview','cal-11'], split=false, point=0, selectedHandle=null, snap=true;
  let pollTimer=null, polling=false, acting=false, generation=0, renderedRevision=-1;
  const filters=new Map();
  function notify(message){$('notice').textContent=message || '';}
  async function api(path, data){
    const abort=new AbortController(), timer=setTimeout(()=>abort.abort(),7000);
    try {
      const res=await fetch('/api/console/'+path, {cache:'no-store', signal:abort.signal,
        ...(data ? {method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':token},body:JSON.stringify(data)} : {})});
      const doc=await res.json();
      if(!res.ok || !doc.ok)throw new Error(doc.error || `HTTP ${res.status}`);
      return doc;
    } finally {clearTimeout(timer);}
  }
  async function command(action, extra={}){
    if(acting){notify('An action is pending. Wait for its response before starting another.');return;}
    acting=true;
    try {
      if(!planning)throw new Error('Planning state is unavailable. Refresh the connection first.');
      const result=await api('action',{action,revision:planning.revision,...extra});
      planning=result.planning;
      notify(result.result ? 'Saved draft: '+result.result.path+'\nNot activated: the V4 runtime has not loaded this export.' : '');
      drawPlanning(); updateJob();
    } catch(error){notify(error.message);}
    finally {acting=false;}
  }
  function go(id, pane=0){tabs[pane]=id;generation++;render();poll();}
  function render(){
    if(!catalog)return;
    const panes=$('panes');panes.replaceChildren();panes.classList.toggle('split',split);
    for(let i=0;i<(split?2:1);i++){
      const section=catalog.screens.find(s=>s.id===tabs[i]);
      const pane=el('section',null,'pane');pane.dataset.screen=section.id;
      if(split){const select=el('select',null,'pane-select');select.setAttribute('aria-label',`Pane ${i+1}`);
        for(const s of catalog.screens){const opt=el('option',s.title);opt.value=s.id;select.append(opt);} select.value=section.id;
        select.onchange=()=>go(select.value,i);pane.append(select);}
      pane.append(el('h1',section.title,'screen-title'));
      const info=el('p',section.id.startsWith('cal-') ? 'Navigation does not pass a calibration step. Only measured results may do that.' : 'Current backend observations only. Missing or stale inputs are not healthy readings.','muted');pane.append(info);
      const buttons=el('div',null,'actions');
      for(const b of section.buttons){
        const wrap=el('div',null,'action-wrap'),button=el('button',b.label,'btn');
        button.dataset.control=b.id;button.disabled=b.disabled;button.title=b.reason || '';
        if(b.action==='point')button.setAttribute('aria-pressed',b.label===`P${point}`);
        button.onclick=()=>handle(b,section.id,i);wrap.append(button);
        if(b.disabled)wrap.append(el('small','Backend required — see integration details below.'));
        buttons.append(wrap);
      } pane.append(buttons);
      if(section.fields.length){const fields=el('div',null,'actions');
        for(const f of section.fields){const label=el('label',f.label),input=el(f.type==='select-one'?'select':'input');
          if(input.tagName==='INPUT')input.type=f.type;input.disabled=!f.available;
          if(f.available){input.checked=snap;input.onchange=()=>{snap=input.checked;};}
          else {input.title='Current value unavailable; the runtime adapter must load and validate this field.';input.placeholder='Not loaded';}
          label.append(input);fields.append(label);}pane.append(fields);}
      const reasons=[...new Set(section.buttons.filter(b=>b.reason).map(b=>b.reason))];
      if(reasons.length){const details=el('details');details.append(el('summary','Integration details — unavailable actions'));
        for(const reason of reasons)details.append(el('p',reason,'missing'));pane.append(details);}
      if(section.id==='cal-11'){
        const label=el('label','Import measured lap: x,y per line, venue metres (maximum 6000 points). Camera-to-map feedback must have been off.');
        const input=el('textarea');input.setAttribute('aria-label','Venue lap CSV');input.placeholder='x,y\n0.1,0.2\n…';label.append(input);pane.append(label);
        const button=el('button','Import lap','btn');button.onclick=()=>{
          try{const lines=input.value.trim().split(/\r?\n/).filter(x=>x.trim());if(lines[0]?.trim().toLowerCase()==='x,y')lines.shift();
            if(lines.length>6000)throw new Error('Maximum 6000 points; decimate the recorded lap first.');
            const points=lines.map(line=>{const cells=line.split(',');if(cells.length!==2 || cells.some(c=>!c.trim() || !Number.isFinite(Number(c))))throw new Error('Each lap row must contain two finite numbers: x,y.');return cells.map(Number);});
            command('import_lap',{frame:'venue',points});
          }catch(error){notify(error.message);}};pane.append(button);
      }
      if(['cal-11','cal-12','map'].includes(section.id)){
        if(section.id==='cal-12'){const plan=el('button','Plan all three legs','btn');plan.onclick=()=>command('plan');pane.append(plan);}
        const host=el('div',null,'planning-view');pane.append(host);
      } else {
        const live=el('div',null,'live-data panel');pane.append(live);
      }
      panes.append(pane);
    }
    drawPlanning();renderTelemetry();
  }
  async function handle(button, screen, pane){
    const action=button.action;
    if(action==='navigate'){go(button.navigation,pane);return;}
    if(action==='point'){point=Number(button.label[1]);render();return;}
    if(action==='review'){point=4;render();await command('plan');return;}
    if(action==='full_build'){notify('Full build: import a measured venue lap, Auto-fit, drag handles, Refit, then Save map. The map and recorded lap use different coordinate frames.');return;}
    if(action==='sensor_check'){await poll();notify(telemetry ? 'Observations refreshed. Inspect ages below. This is not a saved calibration PASS.' : 'No ROS backend connected. Run this console with the dashboard to inspect real observations.');return;}
    if(action==='filter'){filters.set(screen,button.label);renderTelemetry();notify('Showing '+button.label.toLowerCase()+' observations. Missing data is listed explicitly.');return;}
    if(action==='rotate' || action==='flip' || action==='reset_pose'){
      if(point>3){notify('Select P0, P1, P2 or P3 before editing a pose.');return;}
      await command('pose',{index:point,edit:action==='reset_pose'?'reset':action,
        degrees:action==='rotate'?Number(button.label.replace('−','-').replace('°','')):undefined});return;
    }
    if(action==='reset_handle' && !selectedHandle){notify('Select a handle on the map first.');return;}
    await command(action,{handle:selectedHandle});
  }
  const NS='http://www.w3.org/2000/svg';
  function svgEl(tag,attrs){const n=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);return n;}
  function venue(p){const [x,y,a]=planning.transform,c=Math.cos(a),s=Math.sin(a);return [x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]];}
  function track(p){const [x,y,a]=planning.transform,c=Math.cos(a),s=Math.sin(a);return [c*(p[0]-x)+s*(p[1]-y),-s*(p[0]-x)+c*(p[1]-y)];}
  function drawPlanning(){
    for(const host of document.querySelectorAll('.planning-view')){
      // Avoid replacing a pointer-captured SVG during a poll.
      if(host.dataset.dragging==='true')continue;
      host.replaceChildren();
      if(!planning){host.append(el('p','Planning data unavailable. Refresh to retry.','missing'));continue;}
      const mode=host.closest('[data-screen]').dataset.screen, isMap=mode==='cal-11';
      host.append(el('p',planning.warning,'missing'));
      if(mode==='map')host.append(el('p','Draft geometry — not the robot’s active map.','missing'));
      const convert=isMap?venue:p=>p,lines=Object.values(planning.lines).map(line=>line.map(convert));
      const pts=[...lines.flat(),...(isMap?planning.lap:[])];
      const xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);
      const minX=Math.min(...xs)-.4,maxX=Math.max(...xs)+.4,minY=Math.min(...ys)-.4,maxY=Math.max(...ys)+.4;
      const svg=svgEl('svg',{class:'draft-scene',viewBox:`${minX} ${-maxY} ${maxX-minX} ${maxY-minY}`,role:'img','aria-label':isMap?'Editable venue lap and track map':'Mission poses and planned routes'});
      const pointString=ps=>ps.map(p=>p[0]+','+(-p[1])).join(' ');
      for(const line of lines){svg.append(svgEl('polyline',{points:pointString(line),class:'roadline'}));svg.append(svgEl('polyline',{points:pointString(line),class:'centreline'}));}
      if(isMap){
        if(planning.lap.length)svg.append(svgEl('polyline',{points:pointString(planning.lap),fill:'none',stroke:'var(--warn)','stroke-width':.018}));
        for(const h of planning.handles){const p=venue(h.point);const dot=svgEl('circle',{cx:p[0],cy:-p[1],r:.055,fill:h.id===selectedHandle?'var(--lane)':'var(--warn)',stroke:'var(--ink)','stroke-width':.012,'data-handle':h.id});
          const title=svgEl('title',{});title.textContent=h.id;dot.append(title);svg.append(dot);}
      }else if(planning.poses){
        for(let i=0;i<3;i++){const route=planning.routes[i];for(const piece of route?.pieces || []){
          const ps=piece.path;for(let j=1;j<ps.length;j++)svg.append(svgEl('line',{x1:ps[j-1][0],y1:-ps[j-1][1],x2:ps[j][0],y2:-ps[j][1],stroke:ps[j][3]<0?'var(--warn)':['var(--lane)','var(--ok)','var(--ink)'][i],'stroke-width':.022}));}}
        planning.poses.forEach((p,i)=>{
          const g=planning.geometry,a=p[2],c=Math.cos(a),s=Math.sin(a);
          const local=[[-g.rear_overhang,-g.width/2],[g.length-g.rear_overhang,-g.width/2],[g.length-g.rear_overhang,g.width/2],[-g.rear_overhang,g.width/2]];
          const corners=local.map(q=>[p[0]+c*q[0]-s*q[1],p[1]+s*q[0]+c*q[1]]);
          svg.append(svgEl('polygon',{points:pointString(corners),fill:planning.fits[i].fits?'var(--ok)':'var(--bad)',stroke:i===point?'var(--lane)':'var(--ink)','stroke-width':.02,'data-pose':i}));
          const label=svgEl('text',{x:p[0],y:-p[1]-.17,'font-size':.12,fill:'var(--ink)'});label.textContent='P'+i;svg.append(label);
          if(i===point)svg.append(svgEl('circle',{cx:p[0]+.42*c,cy:-(p[1]+.42*s),r:.04,fill:'var(--warn)','data-rotate':i}));
        });
      }
      if(mode!=='map')wireDrag(svg,host,isMap);
      host.append(svg);
      host.append(el('p',isMap?'Orange: measured lap. Drag a handle to edit geometry; whole-map fitting changes its venue transform.':'Poses are rear-axle centres. Select P0–P3, drag the body to move it or its orange handle to rotate. Orange routes reverse.'));
      const details=el('pre');details.textContent=JSON.stringify(isMap?planning.report:{poses:planning.poses,fits:planning.fits,legs:planning.routes.map((r,i)=>({leg:i+1,ok:r?.ok??false,reason:r?.reason||'Not planned',sections:r?.sections,roundabout_exits:r?.roundabout_exits})),map_sha1:planning.map_sha1},null,2);host.append(details);
      if(planning.saved_bundle)host.append(el('p','Draft files: '+planning.saved_bundle));
    }
    renderedRevision=planning?.revision??-1;
  }
  function wireDrag(svg,host,isMap){
    let drag=null;
    const coords=e=>{const p=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse());return [p.x,-p.y];};
    svg.onpointerdown=e=>{
      const target=e.target.closest('[data-handle],[data-pose],[data-rotate]');if(!target)return;
      if(planning.job && ['running','stopping'].includes(planning.job.state)){notify('Stop computation before editing the map or poses.');return;}
      const q=coords(e);drag={target,start:q,revision:planning.revision,handle:target.dataset.handle,rotate:target.dataset.rotate,index:Number(target.dataset.pose??target.dataset.rotate)};
      if(isMap)selectedHandle=drag.handle;else if(drag.index!==point){point=drag.index;drag=null;render();return;}
      host.dataset.dragging='true';svg.setPointerCapture(e.pointerId);
    };
    svg.onpointermove=e=>{if(drag){const q=coords(e);drag.target.setAttribute('transform',`translate(${q[0]-drag.start[0]} ${-(q[1]-drag.start[1])})`);}};
    svg.onpointerup=e=>{
      if(!drag)return;const d=drag,q=coords(e);drag=null;host.dataset.dragging='false';
      if(svg.hasPointerCapture(e.pointerId))svg.releasePointerCapture(e.pointerId);
      if(d.revision!==planning.revision){notify('Draft changed while dragging. Review it and try again.');drawPlanning();return;}
      if(Math.hypot(q[0]-d.start[0],q[1]-d.start[1])<.003){drawPlanning();return;}
      if(isMap)command('move_handle',{handle:d.handle,point:track(q)});
      else {const p=planning.poses[d.index],pose=d.rotate!==undefined?[p[0],p[1],Math.atan2(q[1]-p[1],q[0]-p[0])]:[p[0]+q[0]-d.start[0],p[1]+q[1]-d.start[1],p[2]];
        command('pose',{index:d.index,edit:'place',point:pose,snap:d.rotate===undefined && snap});}
    };
    svg.onpointercancel=()=>{drag=null;host.dataset.dragging='false';drawPlanning();};
  }
  function renderTelemetry(){
    for(const target of document.querySelectorAll('.live-data')){
      target.replaceChildren();
      if(!online || !telemetry){target.append(el('p','Live data unavailable. No robot connection is established in the local preview.','missing'));continue;}
      const screen=target.closest('[data-screen]').dataset.screen,filter=filters.get(screen);
      const fresh=telemetry.freshness_sec || {};
      const rows=el('table'),head=el('tr');for(const title of ['Source','Received age','State'])head.append(el('th',title));rows.append(head);
      for(const [source,age] of Object.entries(fresh)){
        const row=el('tr');row.append(el('td',source),el('td',age==null?'Never received':`${age.toFixed(2)} s`),el('td',age==null?'MISSING':age>2?'STALE':'Received',age==null || age>2?'missing':''));rows.append(row);
      }target.append(rows);
      let observations=telemetry.v4_status || {};
      if(screen==='percplan' && filter){const key={Road:'trajectory',Parking:'parking',Recovery:'recovery'}[filter];observations={[key]:observations[key]||'Missing'};}
      if(screen==='events'){
        target.append(el('p',`${filter||'All'} event stream is not connected. ROS event history cannot be reconstructed from the latest status snapshot.`,'missing'));continue;
      }
      const pre=el('pre');pre.textContent=JSON.stringify(observations,null,2);target.append(pre);
    }
  }
  function updateJob(){
    const job=planning?.job,busy=job && ['running','stopping'].includes(job.state);
    $('stopJob').disabled=!busy;$('job').className=busy || job?.error?'missing':'';
    $('job').textContent=job ? `${job.operation}: ${job.state} (${job.elapsed}s)${busy?' — stop this computation before editing, saving or starting another.':''}\n${job.error||''}\n${(job.log||[]).slice(-6).join('\n')}` : 'No computation running.';
  }
  async function poll(){
    clearTimeout(pollTimer);if(polling || document.hidden)return;polling=true;
    const version=generation;
    try{
      const live=await api('status');online=true;telemetry=live.data;
      $('connection').textContent=live.connected?'Backend connected · preview':'Preview only · no ROS';
      $('connection').className='chip '+(live.connected?'ok':'warn');
      const wanted=tabs.slice(0,split?2:1).some(id=>['cal-11','cal-12','map'].includes(id));
      if(wanted || planning?.job?.state==='running' || planning?.job?.state==='stopping'){
        const state=await api('planning');
        // Responses from old tabs cannot remount obsolete content.
        if(version===generation && (!planning || state.planning.revision>=planning.revision)){
          planning=state.planning;if(renderedRevision!==planning.revision)drawPlanning();updateJob();
        }
      }
      renderTelemetry();
    }catch(error){online=false;telemetry=null;$('connection').textContent='Disconnected — live data cleared';$('connection').className='chip bad';renderTelemetry();notify('Connection/error: '+error.message+'\nActions are never automatically replayed. Refresh to retry.');}
    finally{polling=false;pollTimer=setTimeout(poll,1500);}
  }
  async function start(){
    try{
      const doc=await api('catalog');catalog=doc.catalog;token=doc.token;
      const nav=$('nav');nav.replaceChildren();
      for(const s of catalog.screens){const b=el('button',s.title,'navbtn');b.onclick=()=>go(s.id);nav.append(b);}
      const shell=$('shell');shell.replaceChildren();
      for(const b of catalog.shellButtons){
        if(b.tab)continue;
        // Preserve status/navigation shortcuts without showing mock sample values.
        const title=b.navigation?('Open '+catalog.screens.find(s=>s.id===b.navigation)?.title):b.mode?b.label:b.scenario?b.label+' (mock scenario — not a robot command)':b.id==='splitbtn'?'Split view':b.id==='tkCancel'?'Keep autonomous':b.id==='tkGo'?'Take control, 0 marks':b.label.startsWith('E-STOP')?'E-STOP':b.label.startsWith('STOP')?'STOP MOTORS':'Manual control / Hand back';
        const button=el('button',title,'btn');button.onclick=()=>{
          if(b.navigation)go(b.navigation);else if(b.id==='splitbtn')$('split').click();
          else if(b.id==='tkCancel')$('takeover').close();
          else if(b.id==='manualBtn')$('takeover').showModal();
          else notify(b.mode?catalog.gaps.launch:b.scenario?'The original Driving/Stopped controls only selected mock scenarios. Live driving state cannot be changed by a display toggle.':catalog.gaps.motion);
        };shell.append(button);
      }
      for(const id of ['calibrateMode','raceMode'])$(id).onclick=()=>notify(catalog.gaps.launch);
      for(const id of ['estop','motors'])$(id).onclick=()=>notify(catalog.gaps.motion);
      $('manual').onclick=()=>$('takeover').showModal();$('keepAuto').onclick=()=>$('takeover').close();
      render();poll();
    }catch(error){notify('Console failed to initialize: '+error.message+'\nPress Refresh connection to retry.');}
  }
  $('split').onclick=()=>{split=!split;$('split').setAttribute('aria-pressed',split);generation++;render();poll();};
  $('stopJob').onclick=()=>command('cancel',{job_id:planning?.job?.id});
  $('retry').onclick=()=>start();
  document.addEventListener('visibilitychange',()=>{if(document.hidden)clearTimeout(pollTimer);else poll();});
  start();
})();
