let imuCalStep = 0;
function calibrateIMU() {
  const btn = document.getElementById('imuCalBtn');
  const msg = document.getElementById('imuCalMsg');
  
  if (imuCalStep === 0) {
    // Step 1: Zero
    btn.textContent = 'Zero Axes';
    btn.style.background = 'rgba(223,142,29,0.15)';
    btn.style.color = 'var(--warning)';
    msg.innerHTML = '<b>Step 1:</b> Place robot perfectly flat on the ground and click <b>Zero Axes</b>.';
    msg.style.color = 'var(--text)';
    imuCalStep = 1;
    
  } else if (imuCalStep === 1) {
    // Execute Zero -> prompt Pitch
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'zero'}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = 'Set Pitch +90°';
        btn.style.background = 'rgba(30,102,245,0.15)';
        btn.style.color = 'var(--accent)';
        msg.innerHTML = '✅ Axes Zeroed.<br><b>Step 2:</b> Point robot nose straight UP (90°) and click <b>Set Pitch +90°</b>.';
        imuCalStep = 2;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
      
  } else if (imuCalStep === 2) {
    // Execute Pitch -> prompt Roll
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'set_scale', axis: 'pitch', target: 90.0}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = 'Set Roll +90°';
        msg.innerHTML = '✅ Pitch Mapped.<br><b>Step 3:</b> Tilt robot 90° onto its RIGHT side and click <b>Set Roll +90°</b>.';
        imuCalStep = 3;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
      
  } else if (imuCalStep === 3) {
    // Execute Roll -> Finish
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'set_scale', axis: 'roll', target: 90.0}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = '⚙ Calibrate Wizard';
        btn.style.background = 'rgba(30,102,245,0.08)';
        msg.innerHTML = '✅ Calibration Complete!<br><span style="color:var(--warning)">Remember to open the Right Drawer and click <b>Save Defaults</b> to persist.</span>';
        imuCalStep = 0;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
  }
}

