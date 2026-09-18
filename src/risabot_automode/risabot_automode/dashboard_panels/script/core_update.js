let last_data_time = 0;
function update() {
  const fetchStart = performance.now();
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      const latency = Math.round(performance.now() - fetchStart);
      document.getElementById('latencyText').textContent = latency + ' ms';
      document.getElementById('connDot').style.background = '#4caf50';
      document.getElementById('connText').textContent = 'Connected';

      // Stream robustness is handled client-side by startCamStream()
      // (fetch-based MJPEG player with stall detection + reconnect).

      // State
      const sb = document.getElementById('stateBadge');
      const currentState = d.state;
      sb.textContent = currentState;
      sb.className = 'state-badge state-' + currentState;

      // Update competition flow
      updateFlow(currentState);

      // Log state changes
      if (currentState !== lastState && lastState !== '') {
        addLogEntry(`State: <span class="log-val">${lastState}</span> ➔ <span class="log-val">${currentState}</span>`);
      }
      lastState = currentState;

      const mb = document.getElementById('modeBadge');
      const newMode = d.auto_mode ? 'AUTO' : 'MANUAL';
      const oldMode = mb.textContent;
      mb.textContent = newMode;
      mb.className = 'mode-badge mode-' + newMode;
      if (oldMode && oldMode !== newMode) {
        addLogEntry(`Mode: <span class="log-val">${newMode}</span>`);
      }

      document.getElementById('lapBadge').textContent = 'Lap ' + d.lap;
      // Lap timer reset
      if (d.lap !== lastLap && lastLap !== 0) {
        lapStartTime = Date.now();
        addLogEntry(`Lap <span class="log-val">${d.lap}</span> started`);
      }
      lastLap = d.lap;
      document.getElementById('stateTime').textContent = d.state_time;
      document.getElementById('stateDist').textContent = d.state_dist;

      // Stop reason badge
      const stopEl = document.getElementById('stopBadge');
      const sr = d.stop_reason || '';
      if (sr.length > 0) {
        stopEl.textContent = '⛔ ' + sr;
        stopEl.style.background = 'rgba(243,139,168,0.2)';
        stopEl.style.color = '#f38ba8';
        if (sr !== lastStopReason && lastStopReason === '') {
          addLogEntry(`⛔ Stopped: <span class="log-val">${sr}</span>`);
        }
      } else {
        stopEl.textContent = 'DRIVING';
        stopEl.style.background = 'rgba(64,160,43,0.15)';
        stopEl.style.color = '#40a02b';
        if (lastStopReason && lastStopReason.length > 0) {
          addLogEntry(`✅ Resumed driving`);
        }
      }
      lastStopReason = sr;

      // Traffic light
      ['Red','Yellow','Green'].forEach(c => {
        document.getElementById('tl'+c).classList.toggle('active', d.traffic_light === c.toLowerCase());
      });
      document.getElementById('tlText').textContent = d.traffic_light;

      // IMU Attitude (roll/pitch/yaw)
      (function() {
        const roll  = d.imu_roll  || 0;
        const pitch = d.imu_pitch || 0;
        const yaw   = d.imu_yaw   || 0;
        document.getElementById('imuRollVal').textContent  = roll.toFixed(2)  + '°';
        document.getElementById('imuPitchVal').textContent = pitch.toFixed(2) + '°';
        document.getElementById('imuYawVal').textContent   = yaw.toFixed(2)   + '°';
        // Map [-90,90] → [0%,100%] for bars (centre = 50%)
        const toBar = v => Math.min(100, Math.max(0, 50 + (v / 90) * 50));
        document.getElementById('imuRollBar').style.width  = toBar(roll)  + '%';
        document.getElementById('imuPitchBar').style.width = toBar(pitch) + '%';
        // Yaw maps [-180,180] → [0%,100%]
        document.getElementById('imuYawBar').style.width   = Math.min(100, Math.max(0, 50 + (yaw / 180) * 50)) + '%';
        // Colour pitch bar red when steep (>10°)
        const pitchBar = document.getElementById('imuPitchBar');
        if (Math.abs(pitch) > 10) {
          pitchBar.style.background = 'linear-gradient(90deg,#b71c1c,#f44336)';
        } else {
          pitchBar.style.background = 'linear-gradient(90deg,#e65100,#ff9800)';
        }
      })();

      // Sensors
      function ss(dId, vId, v, tl, fl) {
        const dot = document.getElementById(dId), val = document.getElementById(vId);
        if (v===null||v===undefined){dot.className='dot dot-gray';val.textContent='—';return;}
        dot.className = v ? 'dot dot-red' : 'dot dot-green';
        val.textContent = v ? (tl||'YES') : (fl||'NO');
      }
      ss('dotLidar','valLidar',d.lidar_obstacle,'BLOCKED','CLEAR');
      ss('dotCam','valCam',d.camera_obstacle,'BLOCKED','CLEAR');
      ss('dotFused','valFused',d.fused_obstacle,'BLOCKED','CLEAR');
      ss('dotTunnel','valTunnel',d.tunnel_detected,'IN TUNNEL','NO');
      ss('dotObst','valObst',d.obstruction_active,'DODGING','NO');
      ss('dotPark','valPark',d.parking_complete,'DONE','NO');
      ss('dotSignage','valSignage',d.parking_sign_detected,'DETECTED','CLEAR');
      if (d.health_ok === null || d.health_ok === undefined) {
        document.getElementById('dotHealth').className = 'dot dot-gray';
        document.getElementById('valHealth').textContent = '—';
      } else if (d.health_ok) {
        document.getElementById('dotHealth').className = 'dot dot-green';
        document.getElementById('valHealth').textContent = 'OK';
      } else {
        document.getElementById('dotHealth').className = 'dot dot-red';
        document.getElementById('valHealth').textContent = 'STALE';
      }
      const stale = [...new Set([
        ...(Array.isArray(d.health_stale) ? d.health_stale : []),
        ...(Array.isArray(d.stale_streams) ? d.stale_streams : [])
      ])];
      const staleEl = document.getElementById('valStale');
      if (stale.length === 0) {
        staleEl.textContent = 'NONE';
        staleEl.style.color = '#9bd69b';
      } else {
        staleEl.textContent = stale.slice(0, 3).join(', ') + (stale.length > 3 ? ' +' + (stale.length - 3) : '');
        staleEl.style.color = '#f38ba8';
      }

      // Warning Flash Overlay (if blocked in AUTO)
      const isBlocked = d.fused_obstacle; // Use fused for reliable alarm
      const overlay = document.getElementById('warning-overlay');
      if (overlay) {
        if (d.auto_mode && isBlocked) overlay.classList.add('active');
        else overlay.classList.remove('active');
      }

      const gD=document.getElementById('dotGate'),gV=document.getElementById('valGate');
      if(d.boom_gate===null){gD.className='dot dot-gray';gV.textContent='—';}
      else{gD.className=d.boom_gate?'dot dot-green':'dot dot-red';gV.textContent=d.boom_gate?'OPEN':'CLOSED';}

      // Lane
      document.getElementById('laneErr').textContent = d.lane_error.toFixed(3);
      document.getElementById('laneBar').style.width = Math.min(Math.max((d.lane_error+0.5)/1.0*100,0),100)+'%';
      document.getElementById('cmdLinX').textContent = d.cmd_lin_x.toFixed(3);
      document.getElementById('cmdAngZ').textContent = d.cmd_ang_z.toFixed(3);

      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3)+' m/s';
      document.getElementById('odomX').textContent = (d.odom_x || 0).toFixed(2) + ' m';
      document.getElementById('odomY').textContent = (d.odom_y || 0).toFixed(2) + ' m';
      document.getElementById('odomYaw').textContent = ((d.odom_yaw || 0) * 180 / Math.PI).toFixed(1) + '°';

      // Speed & Selector
      document.getElementById('speedPct').textContent = d.speed_pct+'%';
      document.getElementById('speedBar').style.width = d.speed_pct+'%';
      // Update gear dots
      const gears = [25,40,60,100];
      gears.forEach((g,i) => {
        const dot = document.getElementById('gear'+i);
        if(dot) dot.classList.toggle('active', d.speed_pct >= g);
      });
      document.getElementById('ctrlState').textContent = d.ctrl_state_name;

      // Controller buttons — Updated to match user's specific mapping:
      // A=0, B=1, X=3, Y=4, LB=6, RB=7, Start=11
      const bm={0:'btnA',1:'btnB',3:'btnX',4:'btnY',6:'btnLB',7:'btnRB',8:'btnLT',9:'btnRT',11:'btnStart'};
      Object.values(bm).forEach(id=>document.getElementById(id).classList.remove('active'));
      if(d.buttons){
        d.buttons.forEach((v,i)=>{if(v&&bm[i])document.getElementById(bm[i]).classList.add('active');});
        const p=[];d.buttons.forEach((v,i)=>{if(v)p.push('btn['+i+']');});
        const db=document.getElementById('btnDebug');
        if(p.length){db.textContent='Active: '+p.join(', ');db.style.color='var(--success)';}
        else{db.textContent='No buttons pressed';db.style.color='var(--muted)';}
      }

      // D-pad (axes 6,7)
      if(d.axes&&d.axes.length>7){
        document.getElementById('btnLeft').classList.toggle('active',d.axes[6]>0.5);
        document.getElementById('btnRight').classList.toggle('active',d.axes[6]<-0.5);
        document.getElementById('btnUp').classList.toggle('active',d.axes[7]>0.5);
        document.getElementById('btnDown').classList.toggle('active',d.axes[7]<-0.5);
      }
      if(d.axes&&d.axes.length>3){
        document.getElementById('joyValL').textContent='L: '+d.axes[0].toFixed(1)+', '+d.axes[1].toFixed(1);
        document.getElementById('joyValR').textContent='R: '+d.axes[2].toFixed(1)+', '+d.axes[3].toFixed(1);
        
        let lx = 50 + (d.axes[0] * -50);
        let ly = 50 + (d.axes[1] * -50);
        let rx = 50 + (d.axes[2] * -50);
        let ry = 50 + (d.axes[3] * -50);
        
        const dl = document.getElementById('joyDotL');
        const dr = document.getElementById('joyDotR');
        
        dl.style.left = lx + '%';
        dl.style.top = ly + '%';
        dl.classList.toggle('active', Math.abs(d.axes[0]) > 0.05 || Math.abs(d.axes[1]) > 0.05);
        
        dr.style.left = rx + '%';
        dr.style.top = ry + '%';
        dr.classList.toggle('active', Math.abs(d.axes[2]) > 0.05 || Math.abs(d.axes[3]) > 0.05);
      }

      // ── Record & Playback State ──
      const rpState = d.rp_state || 'IDLE';
      const bufSize = d.rp_buffer_size || 0;
      const pbIdx = d.rp_playback_index || 0;

      const badge = document.getElementById('rpStateBadge');
      if (badge) {
        badge.textContent = rpState;
        badge.className = 'rp-state-badge ' + rpState.toLowerCase();
      }

      const bufSizeEl = document.getElementById('rpBufferSize');
      if (bufSizeEl) bufSizeEl.textContent = bufSize;
      
      const durEl = document.getElementById('rpDuration');
      if (durEl) durEl.textContent = (bufSize * 0.05).toFixed(1);

      const btnRec = document.getElementById('rpBtnRecord');
      const btnStop = document.getElementById('rpBtnStop');
      const btnPlay = document.getElementById('rpBtnPlay');
      const btnSave = document.getElementById('rpBtnSave');

      if (btnRec) {
        btnRec.classList.toggle('active', rpState === 'RECORDING');
        btnRec.disabled = (rpState === 'PLAYBACK');
      }
      if (btnPlay) {
        btnPlay.classList.toggle('active', rpState === 'PLAYBACK');
        btnPlay.disabled = (rpState === 'RECORDING' || bufSize === 0);
      }
      if (btnStop) {
        btnStop.disabled = (rpState === 'IDLE');
      }
      if (btnSave) {
        btnSave.disabled = (rpState !== 'IDLE' || bufSize === 0);
      }

      const progressTrack = document.getElementById('rpProgress');
      const progressFill = document.getElementById('rpProgressFill');
      if (progressTrack && progressFill) {
        if (rpState === 'PLAYBACK' && bufSize > 0) {
          progressTrack.style.display = 'block';
          const pct = Math.min(100, (pbIdx / bufSize) * 100);
          progressFill.style.width = pct + '%';
        } else {
          progressTrack.style.display = 'none';
          progressFill.style.width = '0%';
        }
      }
    })
    .catch(()=>{
      document.getElementById('connDot').style.background='#f44336';
      document.getElementById('connText').textContent='Disconnected';
    });
}
// ===== PARAMETER TUNING (curated) =====
