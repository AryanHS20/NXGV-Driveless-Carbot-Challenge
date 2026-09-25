// Dashboard /data contains hardware odometry and its receive age. This is an
// estimated pose, never a measurement of the car's true course position.
export function dashboardOdom(data, maxAgeSec = 0.5) {
  const age = data?.freshness_sec?.odom;
  const x = data?.odom_x, y = data?.odom_y, a = data?.odom_yaw;
  if (typeof age !== 'number' || !Number.isFinite(age) || age < 0 || age > maxAgeSec ||
      ![x, y, a].every(v => typeof v === 'number' && Number.isFinite(v))) return null;
  const speed = typeof data.speed === 'number' && Number.isFinite(data.speed) ? data.speed : 0;
  return {x, y, a, speed, age};
}

export function alignAtStart(odom, start) {
  const rotation = start.a - odom.a;
  return {odomX: odom.x, odomY: odom.y, startX: start.x, startY: start.y, rotation};
}

export function coursePose(odom, alignment) {
  const dx = odom.x - alignment.odomX, dy = odom.y - alignment.odomY;
  const cos = Math.cos(alignment.rotation), sin = Math.sin(alignment.rotation);
  return {
    x: alignment.startX + cos * dx - sin * dy,
    y: alignment.startY + sin * dx + cos * dy,
    a: Math.atan2(Math.sin(odom.a + alignment.rotation), Math.cos(odom.a + alignment.rotation)),
    speed: odom.speed,
  };
}
