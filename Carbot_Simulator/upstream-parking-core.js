/* Browser-side port of Aquadrox parking_controller.py, blob 5ebef7aaaa281d04.
   Source: https://github.com/Aquadrox-Technologies/NXGV-Driveless-Carbot-Challenge/blob/c6321539fee7961f2cf9023b6fe50f652eed59da/src/risabot_automode/risabot_automode/parking_controller.py
   Phase commands and thresholds follow the source; plant/geometry are explicit
   simulation assumptions and are not part of that ROS node. */
(() => {
  'use strict';
  const DT = 0.05;
  const DEFAULTS = Object.freeze({
    parallel_forward_dist: 0.30,
    parallel_reverse_dist: 0.35,
    parallel_steer_angle: 0.6,
    perp_turn_angle: 1.57,
    perp_forward_dist: 0.25,
    park_wait_time: 3.0,
    drive_speed: 0.15,
    reverse_speed: -0.12,
  });
  const PHASES = Object.freeze({
    IDLE: 'IDLE',
    PARALLEL_FORWARD: 'PARALLEL_FORWARD',
    PARALLEL_STEER_REVERSE: 'PARALLEL_STEER_REVERSE',
    PARALLEL_STRAIGHTEN: 'PARALLEL_STRAIGHTEN',
    PARALLEL_WAIT: 'PARALLEL_WAIT',
    PARALLEL_EXIT: 'PARALLEL_EXIT',
    PERP_TURN_IN: 'PERP_TURN_IN',
    PERP_FORWARD: 'PERP_FORWARD',
    PERP_WAIT: 'PERP_WAIT',
    PERP_REVERSE_OUT: 'PERP_REVERSE_OUT',
  });
  const geometry = Object.freeze({wheelbase: 0.21, front: 0.233, rear: 0.042, halfWidth: 0.0925});

  function corners(pose) {
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    return [[geometry.front, geometry.halfWidth], [geometry.front, -geometry.halfWidth],
      [-geometry.rear, -geometry.halfWidth], [-geometry.rear, geometry.halfWidth]]
      .map(([along, side]) => ({x: pose.x + along * c - side * s,
        y: pose.y + along * s + side * c}));
  }
  function fits(pose, bay) {
    return corners(pose).every(p => p.x >= bay.x0 && p.x <= bay.x1 &&
      p.y >= bay.y0 && p.y <= bay.y1);
  }

  function simulate(kind, options = {}) {
    if (kind !== 'parallel' && kind !== 'perpendicular') throw new Error('Unknown parking kind');
    const params = {...DEFAULTS, ...(options.params || {})};
    const start = options.start || {x: 0.05, y: -0.18, yawDeg: 0};
    const maxWheelDeg = Number(options.maxWheelDeg ?? 50);
    const bayDepth = Number(options.bayDepth ?? 0.40);
    if (![start.x, start.y, start.yawDeg, maxWheelDeg, bayDepth].every(Number.isFinite) ||
        maxWheelDeg <= 0 || maxWheelDeg >= 80 || bayDepth <= 0.20 || bayDepth > 1.0)
      throw new Error('Invalid starting pose, steering limit or bay depth');
    // Upstream challenge guide gives 0.75 m for parallel and 0.40 m width
    // for perpendicular. The second bay dimension is an adjustable assumption.
    const bay = kind === 'parallel'
      ? {x0: 0, x1: 0.75, y0: 0, y1: bayDepth}
      : {x0: 0, x1: 0.40, y0: 0, y1: bayDepth};
    let pose = {x: Number(start.x), y: Number(start.y), yaw: start.yawDeg * Math.PI / 180};
    let phase = kind === 'parallel' ? PHASES.PARALLEL_FORWARD : PHASES.PERP_TURN_IN;
    let time = 0, distance = 0, phaseStartTime = 0, phaseStartDist = 0;
    let parked = null, finish = null;
    const samples = [{time, ...pose, phase, linear: 0, angular: 0, distance}];
    const transitions = [{time, phase}];
    const change = next => {
      phase = next; phaseStartTime = time; phaseStartDist = distance;
      transitions.push({time, phase: next});
      if (next === PHASES.PARALLEL_WAIT || next === PHASES.PERP_WAIT) {
        parked = {time, pose: {...pose}, fits: fits(pose, bay)};
      }
    };

    for (let tick = 0; tick < 1200 && phase !== PHASES.IDLE; tick++) {
      const dist = distance - phaseStartDist;
      const elapsed = time - phaseStartTime;
      let linear = 0, angular = 0;
      // Keep the upstream branch order: a threshold crossing still publishes
      // that phase's command for one 20 Hz tick, then moves to the next phase.
      switch (phase) {
        case PHASES.PARALLEL_FORWARD:
          linear = params.drive_speed;
          if (dist >= params.parallel_forward_dist) change(PHASES.PARALLEL_STEER_REVERSE);
          break;
        case PHASES.PARALLEL_STEER_REVERSE:
          linear = params.reverse_speed; angular = -params.parallel_steer_angle;
          if (dist >= params.parallel_reverse_dist) change(PHASES.PARALLEL_STRAIGHTEN);
          break;
        case PHASES.PARALLEL_STRAIGHTEN:
          linear = params.drive_speed * 0.5; angular = params.parallel_steer_angle * 0.5;
          if (dist >= 0.10) change(PHASES.PARALLEL_WAIT);
          break;
        case PHASES.PARALLEL_WAIT:
          if (elapsed >= params.park_wait_time) change(PHASES.PARALLEL_EXIT);
          break;
        case PHASES.PARALLEL_EXIT:
          linear = params.drive_speed; angular = params.parallel_steer_angle * 0.5;
          if (dist >= 0.30) { finish = {time, pose: {...pose}}; change(PHASES.IDLE); linear = 0; angular = 0; }
          break;
        case PHASES.PERP_TURN_IN:
          angular = 0.5;
          if (elapsed >= params.perp_turn_angle / 0.5) change(PHASES.PERP_FORWARD);
          break;
        case PHASES.PERP_FORWARD:
          linear = params.drive_speed;
          if (dist >= params.perp_forward_dist) change(PHASES.PERP_WAIT);
          break;
        case PHASES.PERP_WAIT:
          if (elapsed >= params.park_wait_time) change(PHASES.PERP_REVERSE_OUT);
          break;
        case PHASES.PERP_REVERSE_OUT:
          linear = params.reverse_speed;
          if (dist >= params.perp_forward_dist) { finish = {time, pose: {...pose}}; change(PHASES.IDLE); linear = 0; angular = 0; }
          break;
      }
      // Upstream servo_controller maps angular.z to a steering servo, even
      // when linear.x=0. Ackermann yaw therefore remains zero at zero speed.
      // Its positive command moves the physical servo right; use negative
      // mathematical yaw for right turns in this top-down coordinate system.
      const wheelAngle = -Math.max(-1, Math.min(1, angular)) * maxWheelDeg * Math.PI / 180;
      const nextYaw = pose.yaw + linear / geometry.wheelbase * Math.tan(wheelAngle) * DT;
      pose = {
        x: pose.x + linear * Math.cos((pose.yaw + nextYaw) / 2) * DT,
        y: pose.y + linear * Math.sin((pose.yaw + nextYaw) / 2) * DT,
        yaw: nextYaw,
      };
      distance += Math.abs(linear) * DT; // upstream odom_callback integrates abs(speed)
      time = Math.round((time + DT) * 100) / 100;
      samples.push({time, ...pose, phase, linear, angular, distance});
    }
    return {kind, sourceBlob: '5ebef7aaaa281d04fb2e5bb888b9fc6e14256edf',
      params, bay, start, geometry, samples, transitions, parked, finish,
      startsInside: fits(samples[0], bay), complete: Boolean(finish), duration: time};
  }

  window.UpstreamParkingCore = {simulate, corners, fits, DEFAULTS, PHASES, DT};
})();
