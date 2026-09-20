const PARAM_TIPS = {
  // Traffic light
  red_h_low1:'Red hue range 1 lower bound (HSV)', red_h_high1:'Red hue range 1 upper bound',
  red_h_low2:'Red hue range 2 lower bound (wrap)', red_h_high2:'Red hue range 2 upper bound',
  yellow_h_low:'Yellow hue lower', yellow_h_high:'Yellow hue upper',
  green_h_low:'Green hue lower', green_h_high:'Green hue upper',
  sat_min:'Min saturation to count as color', val_min:'Min brightness to count as color',
  min_circle_radius:'Smallest circle to detect', max_circle_radius:'Largest circle to detect',
  min_pixel_count:'Min colored pixels to trigger', required_confidence:'Consecutive detections required',
  resize_width:'Resize width for CV speed', heartbeat_sec:'Publish heartbeat period',
  // Line follower (MDPI-enhanced scanline)
  n_scanlines:'Number of horizontal scanlines to sample', min_valid_scanlines:'Min scanlines for confident lock',
  min_line_width_px:'Min white region width in pixels (noise filter)',
  crop_ratio_base:'Bottom crop ratio — how much of the frame is road',
  white_threshold:'Fixed gray threshold for binary (used when use_otsu=false)',
  use_otsu:'Use Otsu auto-threshold instead of fixed white_threshold',
  invert_binary:'Invert binary image — detect dark lane (true) or white borders (false)',
  morph_open_size:'Morphological OPEN kernel size — removes small noise blobs (0=disable, 3=default)',
  morph_close_size:'Morphological CLOSE kernel size — fills small gaps in lines (0=disable, 5=default)',
  search_radius_px:'Blob-to-expected-position match radius in pixels',
  clahe_enabled:'Enable CLAHE adaptive lighting normalization', clahe_clip_limit:'CLAHE contrast clip limit',
  // IPM (Birds Eye View)
  ipm_enabled:'Enable Birds Eye View perspective warp (MDPI)', ipm_top_width_ratio:'Narrow end of trapezoid (0.2-0.5, lower=stronger warp)',
  ipm_bottom_width_ratio:'Wide end of trapezoid (usually 1.0)',
  // Kalman filter
  kalman_enabled:'Use Kalman filter instead of EMA for lane smoothing',
  kalman_process_noise:'Q — how much the filter trusts its model (lower=smoother)',
  kalman_measurement_noise:'R — how much the filter trusts measurements (lower=more reactive)',
  // Legacy EMA
  smoothing_alpha:'EMA smoothing (0=smooth, 1=raw) — used when Kalman disabled', dead_zone:'Tolerance threshold — ignore error below this',
  hold_error_frames:'Frames to hold last known error when lines lost',
  error_decay_rate:'Per-frame decay for held error (0.92 = halves in ~9 frames)',
  debug_print_rate:'Seconds between console debug prints',
  // Auto driver
  forward_speed:'Max forward speed (m/s) — on a straight',
  stale_timeout:'Seconds before module data is stale', dist_lap_complete:'Distance after green to mark lap',
  enable_subsumption_obstacle:'Enable fused obstacle reverse adjust', max_odom_speed:'Ignore odom speed spikes above this',
  min_state_dwell_sec:'Minimum hold time for non-emergency behavior switches',
  publish_loop_stats:'Enable loop timing diagnostics stream',
  // PID Steering
  pid_kp:'[PID] Proportional gain — how hard to steer. Too high → oscillation/weaving',
  pid_ki:'[PID] Integral gain — corrects long-term drift. Keep very small (0.0–0.05)',
  pid_kd:'[PID] Derivative gain — dampens oscillation. Too high → jittery steering',
  pid_integral_max:'[PID] Anti-windup clamp for I term. Prevents integral runaway in long turns',
  speed_error_scale:'Adaptive speed: higher = robot slows more in turns (try 1.0–2.5)',
  min_turn_speed:'Adaptive speed: minimum speed multiplier in a sharp turn (0.4 = 40% of max)',
  lane_steer_slew:'Max steering change per second — lower = smoother but slower response (Cytron accel limit)',
  // Command safety
  publish_hz:'Safety controller publish loop frequency',
  cmd_timeout:'Max age for raw auto commands before forced zero',
  max_linear_speed:'Clamp for linear auto command speed',
  max_angular_speed:'Clamp for angular auto command speed',
  max_linear_accel:'Linear acceleration slew limit',
  max_angular_accel:'Angular acceleration slew limit',
  deadband_linear:'Zero small linear command noise',
  deadband_angular:'Zero small angular command noise',
  // Tunnel wall follower (RANSAC enhanced)
  target_center_dist:'Offset from center between walls (0=centered)',
  kp_heading:'Proportional gain for heading alignment (wall angle correction)',
  kd_heading:'Derivative gain for heading alignment (dampens heading oscillation)',
  ransac_threshold:'RANSAC inlier distance threshold in meters (lower=stricter fit)',
  ransac_iterations:'RANSAC max iterations (50 is fine for <100 LiDAR pts)',
  tunnel_hysteresis_frames:'Consecutive frames required before toggling tunnel on/off (prevents flicker)',
  min_wall_points:'Min LiDAR points on each side to consider a wall present',
  max_wall_dist:'Ignore LiDAR points beyond this distance (m)',
  // Boom gate
  min_detect_dist:'Closest gate detection (m)', max_detect_dist:'Farthest gate detection (m)',
  angle_window:'Forward arc width (rad)', min_gate_points:'Min LiDAR points for gate',
  distance_variance_max:'Max spread of gate points', lidar_angle_offset:'LiDAR mount rotation (rad)',
  hysteresis:'Consecutive frames before state change',
  // Obstruction
  detect_dist:'Start dodging at this distance (m)', clear_dist:'Consider clear beyond this (m)',
  front_angle:'Front detection arc (rad)', side_angle_min:'Side arc start (rad)', side_angle_max:'Side arc end (rad)',
  steer_speed:'Speed while steering around (m/s)', steer_angular:'Turn rate while dodging (rad/s)',
  pass_speed:'Speed while passing alongside (m/s)', pass_duration:'Seconds driving alongside',
  steer_back_duration:'Seconds steering back to lane', steer_away_duration:'Seconds steering away',
  // Parking
  parallel_forward_dist:'Drive past slot distance (m)', parallel_reverse_dist:'Reverse into slot distance (m)',
  parallel_steer_angle:'Steering rate during reverse (rad/s)', perp_turn_angle:'Turn angle into slot (rad)',
  perp_forward_dist:'Drive into slot distance (m)', park_wait_time:'Seconds to wait in slot',
  drive_speed:'Parking drive speed (m/s)', reverse_speed:'Parking reverse speed (m/s)',
  signboard_min_area:'Min contour area for parking sign', signboard_resize_width:'Resize width for sign detection',
  // Camera obstacle
  edge_threshold:'Edge pixel ratio to trigger (0.0-1.0)', canny_low:'Canny lower threshold',
  canny_high:'Canny upper threshold', blur_kernel:'Gaussian blur kernel (odd number)',
  hysteresis_on:'Frames to confirm obstacle', hysteresis_off:'Frames to confirm clear',
  // Dashboard
  use_hw_odom:'Use hardware encoder odometry', freshness_stale_sec:'Dashboard stale threshold in seconds',
  cam_encode_max_hz:'Max MJPEG encode rate in Hz (lower = less robot CPU)',
  dashboard_port:'HTTP port (takes effect on restart)',
  sim_odom_scale:'Scale for simulated odometry distance', hw_odom_scale:'Scale for hardware odometry distance',
  hw_odom_yaw_scale:'Scale for hardware odometry yaw',
  // Servo controller
  servo_center:'Servo neutral angle (90=default). Adjust if robot drifts left/right when steering is centered',
  servo_range_left:'Left steering range from center (50=default). Increase for sharper left turns',
  servo_range_right:'Right steering range from center (50=default). Increase for sharper right turns',
  servo_steer_id:'Servo channel for steering (4=default)',
  auto_cmd_timeout:'Auto cmd stream timeout before forcing manual stop',
  unlock_requires_neutral:'Require neutral sticks after unlock before driving',
  unlock_neutral_threshold:'Neutral stick threshold for unlock gate',
  ticks_per_meter:'Encoder ticks per meter', odom_distance_scale:'Extra distance calibration multiplier',
  drive_motor_index:'Which motor encoder channel to use (0=FL 1=FR 2=RL 3=RR)',
  // Challenge sequencing
  dist_post_obstacle_clear:'Lane-follow distance after obstacle clears before entering roundabout (m)',
  dist_roundabout:'Distance to traverse the roundabout arc before exiting (m)',
  odom_yaw_scale:'Extra yaw calibration multiplier', encoder_jump_threshold:'Reject tick jumps above this',
  max_linear_velocity:'Clamp linear odom speed', max_angular_velocity:'Clamp angular odom speed',
  odom_reverse_polarity:'Invert encoder odom sign if needed', odom_velocity_deadband:'Zero odom near standstill',
  // Health monitor
  publish_period:'Health publish period (s)', timeout_perception:'Perception timeout (s)',
  timeout_state:'State timeout (s)', timeout_control:'Control timeout (s)',
  timeout_odom:'Odometry timeout (s)', timeout_joy:'Joystick timeout (s)',
  show_debug:'Publish annotated debug frame', print_debug:'Enable console debug printing',
  // Signage detector (BPU)
  model_path:'Path to compiled BPU .bin model file',
  conf_threshold:'Global YOLO confidence fallback (0.0–1.0)',
  iou_threshold:'NMS IoU threshold (0.0–1.0)',
  min_parking_sign_width:'Min pixel width for parking sign trigger (0 = disabled)',
  // Per-class thresholds (YOLO11n NXGV)
  thresh_end_tunnel:'Class 0 end-of-tunnel threshold', thresh_hill:'Class 1 hill sign threshold',
  thresh_obstacle:'Class 2 obstacle sign threshold', thresh_parallelp:'Class 3 parallel parking threshold',
  thresh_perpendp:'Class 4 perpendicular parking threshold', thresh_roundabout:'Class 5 roundabout threshold',
  thresh_speedbump:'Class 6 speed bump threshold', thresh_tl_lamp:'Class 7 traffic lamp threshold',
  thresh_tl_warn:'Class 8 warning sign threshold', thresh_tunnel:'Class 9 tunnel sign threshold',
  tunnel_publish_enabled:'Publish the advisory tunnel sign hint'
};
const PARAM_GROUPS = [
  { node: 'line_follower_camera', label: 'Line Follower', params: [
    'tracker_mode','s_thresh','l_thresh','sobel_thresh','sliding_windows','sliding_margin','sliding_minpix',
    'pp_metric_enabled','pp_lateral_span_m','pp_forward_span_m','pp_near_m','pp_lookahead_ratio','pp_steering_gain',
    'n_scanlines','min_valid_scanlines','min_line_width_px',
    'crop_ratio_base','search_radius_px',
    'white_threshold','use_otsu','invert_binary',
    'morph_open_size','morph_close_size',
    'clahe_enabled','clahe_clip_limit',
    'ipm_enabled','ipm_top_width_ratio','ipm_bottom_width_ratio',
    'kalman_enabled','kalman_process_noise','kalman_measurement_noise',
    'smoothing_alpha','dead_zone','hold_error_frames','error_decay_rate',
    'ground_mask_enabled','ground_mask_tol_m','cam_height_m','cam_tilt_deg','depth_max_age',
    'resize_width','print_debug','debug_print_rate','show_debug'
  ]},
  { node: 'auto_driver', label: 'Auto Driver', params: [
    'forward_speed','stale_timeout','lane_control_mode','parking_idle_duration','parking_max_duration',
    'dist_lap_complete','enable_subsumption_obstacle','max_odom_speed',
    'min_state_dwell_sec','publish_loop_stats',
    'pid_kp','pid_ki','pid_kd','pid_integral_max',
    'speed_error_scale','min_turn_speed','lane_steer_slew',
    'hill_pitch_threshold','hill_base_speed','hill_max_speed'
  ]},
  { node: 'cmd_safety_controller', label: 'Cmd Safety', params: [
    'publish_hz','cmd_timeout','sensor_timeout','min_scan_points','require_signage',
    'footprint_half_width','footprint_front','footprint_rear','sweep_distance','lidar_angle_offset',
    'max_linear_speed','max_angular_speed',
    'max_linear_accel','max_angular_accel','deadband_linear','deadband_angular','publish_loop_stats'
  ]},
  { node: 'boom_gate_detector', label: 'Boom Gate', params: [
    'min_detect_dist','max_detect_dist','angle_window',
    'min_gate_points','distance_variance_max','lidar_angle_offset','hysteresis','heartbeat_sec'
  ]},
  { node: 'tunnel_wall_follower', label: 'Tunnel', params: [
    'target_center_dist','forward_speed','kp','kd','kp_heading','kd_heading','max_angular',
    'left_angle_min','left_angle_max','right_angle_min','right_angle_max',
    'min_wall_points','max_wall_dist','lidar_angle_offset',
    'output_alpha','tunnel_hysteresis_frames','heartbeat_sec'
  ]},
  { node: 'obstruction_avoidance', label: 'Obstruction', params: [
    'detect_dist','min_safe_dist','lateral_offset_m','spline_length_m','forward_speed',
    'lookahead_m','sensor_timeout','max_timeout_sec','lidar_angle_offset'
  ]},
  { node: 'parking_controller', label: 'Parking', params: [
    'parallel_forward_dist','parallel_reverse_dist','parallel_steer_angle',
    'perp_turn_angle','perp_forward_dist','park_wait_time',
    'drive_speed','reverse_speed','lidar_stop_dist','lidar_angle_offset'
  ]},
  { node: 'obstacle_avoidance_camera', label: 'Camera Obstacle', params: [
    'edge_threshold','canny_low','canny_high','blur_kernel',
    'hysteresis_on','hysteresis_off','resize_width','heartbeat_sec','show_debug'
  ]},
  { node: 'obstacle_avoidance_node', label: 'LiDAR Obstacle', params: [
    'min_obstacle_distance','lidar_angle_offset','heartbeat_sec'
  ]},
  { node: 'servo_controller', label: 'Servo/Odom', params: [
    'parallel_recording','perpendicular_recording','motor_duty_per_mps','auto_right_steer_boost',
    'servo_center','servo_range_left','servo_range_right','servo_steer_id',
    'joy_timeout','auto_cmd_timeout','unlock_requires_neutral','unlock_neutral_threshold',
    'ticks_per_meter','drive_motor_index','odom_distance_scale','odom_yaw_scale',
    'encoder_jump_threshold','max_linear_velocity','max_angular_velocity',
    'wheel_base','steering_max_deg','odom_vel_alpha','odom_velocity_deadband',
    'odom_reverse_polarity','publish_loop_stats'
  ]},
  { node: 'dashboard', label: 'Dashboard', params: [
    'use_hw_odom','freshness_stale_sec','sim_odom_scale','hw_odom_scale','hw_odom_yaw_scale',
    'cam_encode_max_hz','dashboard_port'
  ]},
  { node: 'health_monitor', label: 'Health Monitor', params: [
    'publish_period','timeout_perception','timeout_state',
    'timeout_control','timeout_odom','timeout_joy'
  ]},
  { node: 'map_recorder', label: 'Map Recorder', params: [
    'map_enabled','map_dir','sample_hz'
  ]},
  { node: 'mipi_relay', label: 'MIPI Relay', params: [
    'source_topic','target_topic','second_enabled',
    'second_source','second_target','max_hz'
  ]},
  { node: 'signage_detector', label: 'Signage Detector (BPU)', params: [
    'model_path','observation_timeout','conf_threshold','iou_threshold',
    'min_parking_sign_width','heartbeat_sec','show_debug',
    'tunnel_publish_enabled',
    'thresh_end_tunnel','thresh_hill','thresh_obstacle',
    'thresh_parallelp','thresh_perpendp','thresh_roundabout',
    'thresh_speedbump','thresh_tl_lamp','thresh_tl_warn','thresh_tunnel',
    'thresh_traffic_red','thresh_traffic_yellow','thresh_traffic_green',
    'thresh_boom_closed','thresh_boom_partial','thresh_boom_open',
    'tunnel_publish_enabled','publish_boom_state'
  ]},
];

function toggleNodeParams(headerEl, nodeName) {
  const body = headerEl.nextElementSibling;
  const opening = !body.classList.contains('open');
  body.classList.toggle('open');
  if (opening) {
    const group = PARAM_GROUPS.find(g => g.node === nodeName);
    if (group) {
      group.params.forEach(p => {
        getParam(nodeName, p, true);
      });
    }
  }
}

function buildParamUI() {
  const c = document.getElementById('paramContainer');
  c.innerHTML = PARAM_GROUPS.map(g => {
    const rows = g.params.map(p => {
      const tip = PARAM_TIPS[p] || '';
      return `<div class="param-row">
        <span class="param-name" ${tip ? 'title="'+tip+'"' : ''}>${p}</span>
        <input class="param-val" id="pv_${g.node}_${p}" placeholder="—" />
        <button class="param-get-btn" onclick="getParam('${g.node}','${p}')">Get</button>
        <button class="param-set-btn" onclick="setParam('${g.node}','${p}')">Set</button>
        <span class="param-status" id="ps_${g.node}_${p}"></span>
      </div>`;
    }).join('');
    return `<div class="param-node-block">
      <div class="param-node-header" onclick="toggleNodeParams(this, '${g.node}')">
        <span class="param-node-name">${g.label}</span>
        <span class="param-node-count">${g.params.length} params</span>
      </div>
      <div class="param-node-body">${rows}</div>
    </div>`;
  }).join('');
}

async function getParam(node, param, isInitialLoad=false) {
  const status = document.getElementById('ps_' + node + '_' + param);
  const input = document.getElementById('pv_' + node + '_' + param);
  if (status && !isInitialLoad) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/get_param?node=' + encodeURIComponent(node) + '&param=' + encodeURIComponent(param));
    const d = await r.json();
    if (d.ok) {
      if (input) {
        input.value = d.value;
        if (isInitialLoad) {
          const defVal = d.default !== undefined ? d.default : d.value;
          input.placeholder = `Def: ${defVal}`;
          const pName = input.parentElement.querySelector('.param-name');
          if (pName) {
            pName.innerHTML = `${param} <span style="color:#555;font-size:0.85em;">(def: ${defVal})</span>`;
          }
        }
      }
      if (status && !isInitialLoad) { status.className = 'param-status ok'; status.textContent = '✓'; }
    } else {
      if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = d.error || 'Not found'; }
    }
  } catch(e) {
    if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  if (!isInitialLoad) {
    setTimeout(() => { if(status) status.textContent = ''; }, 3000);
  }
}

async function setParam(node, param) {
  const input = document.getElementById('pv_' + node + '_' + param);
  const status = document.getElementById('ps_' + node + '_' + param);
  if (!input || !input.value) { if(status){status.className='param-status err';status.textContent='Empty';} return; }
  if (status) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/set_param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({node: node, param: param, value: input.value})
    });
    const d = await r.json();
    if (d.ok) {
      if (status) { status.className = 'param-status ok'; status.textContent = '✓ Set'; }
    } else {
      if (status) { status.className = 'param-status err'; status.textContent = d.error || 'Failed'; }
    }
  } catch(e) {
    if (status) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  setTimeout(() => { if(status) status.textContent = ''; }, 3000);
}

async function saveDefaults() {
  if (!confirm('Save ALL current runtime parameters as the new defaults in params.yaml?\n\nThis will overwrite the file on disk. You will need to rebuild (colcon build) for the changes to take effect on next launch.')) {
    return;
  }
  const btn = document.getElementById('saveDefaultsBtn');
  const status = document.getElementById('saveDefaultsStatus');
  btn.className = 'param-save-defaults-btn saving';
  btn.textContent = 'â³ Saving...';
  status.textContent = '';
  try {
    const r = await fetch('/api/save_defaults', { method: 'POST' });
    const d = await r.json();
    if (d.ok) {
      btn.className = 'param-save-defaults-btn success';
      btn.textContent = '✓ Saved!';
      status.style.color = 'var(--success)';
      status.textContent = d.msg || `${d.updated} params updated`;
    } else {
      btn.className = 'param-save-defaults-btn error';
      btn.textContent = '✗ Failed';
      status.style.color = 'var(--danger)';
      status.textContent = d.error || 'Unknown error';
    }
  } catch(e) {
    btn.className = 'param-save-defaults-btn error';
    btn.textContent = '✗ Error';
    status.style.color = 'var(--danger)';
    status.textContent = 'Network error';
  }
  setTimeout(() => {
    btn.className = 'param-save-defaults-btn';
    btn.textContent = '💾 Save Current as Default';
    status.textContent = '';
  }, 5000);
}

buildParamUI();

// Parameters are now fetched on-demand when expanding the sections in the UI.

setInterval(update, 200);
update();

// ── LiDAR 2D Visualization ──
(function(){
  const canvas = document.getElementById('lidarCanvas');
  if(!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const CX = W/2, CY = H/2;
  const SCALE = 200;

  function drawLidar(points, tunnelDetected){
    ctx.clearRect(0,0,W,H);
    // Grid circles
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    [0.2, 0.4, 0.6].forEach(function(r){
      ctx.beginPath(); ctx.arc(CX, CY, r*SCALE, 0, Math.PI*2); ctx.stroke();
    });
    // Distance labels
    ctx.fillStyle = 'rgba(255,255,255,0.15)';
    ctx.font = '10px Inter, sans-serif';
    ctx.textAlign = 'left';
    [0.2, 0.4, 0.6].forEach(function(r){
      ctx.fillText(r.toFixed(1)+'m', CX+r*SCALE+2, CY-2);
    });
    // Tunnel detection windows
    ctx.globalAlpha = 0.08;
    ctx.fillStyle = '#4fc3f7';
    ctx.beginPath(); ctx.moveTo(CX, CY);
    ctx.arc(CX, CY, 0.6*SCALE, (-90-90)*Math.PI/180, (-90-30)*Math.PI/180);
    ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#ef5350';
    ctx.beginPath(); ctx.moveTo(CX, CY);
    ctx.arc(CX, CY, 0.6*SCALE, (-90+30)*Math.PI/180, (-90+90)*Math.PI/180);
    ctx.closePath(); ctx.fill();
    ctx.globalAlpha = 1.0;
    // Forward line
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.setLineDash([4,4]);
    ctx.beginPath(); ctx.moveTo(CX, CY); ctx.lineTo(CX, CY - 0.7*SCALE); ctx.stroke();
    ctx.setLineDash([]);
    // LiDAR points
    if(points && points.length > 0){
      points.forEach(function(p){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        var dist = Math.sqrt(p.x*p.x + p.y*p.y);
        if(dist < 0.3) ctx.fillStyle = '#ff5252';
        else if(dist < 0.5) ctx.fillStyle = '#ffd740';
        else ctx.fillStyle = '#69f0ae';
        ctx.beginPath(); ctx.arc(px, py, 2.5, 0, Math.PI*2); ctx.fill();
      });
    }
    // Centerline path (cyan dots + line)
    if(window._lastCenterline && window._lastCenterline.length > 1){
      ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2; ctx.setLineDash([3,3]);
      ctx.beginPath();
      window._lastCenterline.forEach(function(p, i){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        if(i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke(); ctx.setLineDash([]);
      // Draw center dots
      ctx.fillStyle = '#00e5ff';
      window._lastCenterline.forEach(function(p){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        ctx.beginPath(); ctx.arc(px, py, 3.5, 0, Math.PI*2); ctx.fill();
      });
    }
    // Robot icon
    ctx.fillStyle = '#1e88e5'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(CX, CY - 10); ctx.lineTo(CX - 7, CY + 6); ctx.lineTo(CX + 7, CY + 6);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    // Labels
    ctx.fillStyle = 'rgba(255,255,255,0.3)';
    ctx.font = '9px Inter, sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('FRONT', CX, 14);
    ctx.fillText('L', 12, CY+4);
    ctx.fillText('R', W-12, CY+4);
    // Status
    var el = document.getElementById('lidarStatus');
    if(tunnelDetected) el.innerHTML = '<span style="color:#40a02b;">â— TUNNEL</span>';
    else if(points && points.length > 0) el.innerHTML = '<span style="color:#1e66f5;">â— ' + points.length + ' pts</span>';
    else el.innerHTML = '<span style="color:#888;">â— No data</span>';
  }
  function fetchLidar(){
    fetch('/lidar_data').then(function(r){return r.json();}).then(function(d){
      // Store centerline for drawing
      window._lastCenterline = d.centerline || [];
      drawLidar(d.points || [], d.tunnel || false);
      // Show steer debug overlay
      var el = document.getElementById('lidarDebug');
      if(!el){
        el = document.createElement('div');
        el.id = 'lidarDebug';
        el.style.cssText = 'font:bold 11px JetBrains Mono,monospace; color:#fff; padding:6px 8px; position:absolute; bottom:8px; left:8px; right:8px; background:rgba(0,0,0,0.7); border-radius:8px; display:none;';
        canvas.parentElement.style.position = 'relative';
        canvas.parentElement.appendChild(el);
      }
      if(d.tunnel && d.angular_z !== undefined){
        var dir = d.angular_z > 0.01 ? '← LEFT' : (d.angular_z < -0.01 ? 'RIGHT →' : '↑ STRAIGHT');
        var color = Math.abs(d.angular_z) > 0.3 ? '#ff5252' : '#69f0ae';
        el.innerHTML = 'L:' + (d.left_dist||0).toFixed(2) + 'm  R:' + (d.right_dist||0).toFixed(2) + 'm  lat:' + (d.dist_error||0).toFixed(3) + '  <span style="color:'+color+'">ω:' + (d.angular_z||0).toFixed(2) + ' ' + dir + '</span>';
        el.style.display = 'block';
      } else {
        el.style.display = 'none';
      }
    }).catch(function(){});
  }
  setInterval(fetchLidar, 250);
  fetchLidar();
})();
