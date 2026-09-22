"""Pure control contracts shared by production nodes and offline tests.

angular.z is normalized steering: positive RIGHT, negative LEFT, [-1, 1].
linear.x remains a requested speed; the motor duty map needs bench calibration.
"""
import math


def fresh(stamp, now, timeout):
    return stamp > 0 and 0 <= now - stamp <= timeout


def valid_scan(scan, minimum=20):
    return sum(math.isfinite(r) and scan.range_min < r <= scan.range_max
               for r in scan.ranges) >= minimum


def observation_age(msg, now_seconds):
    """Age of the sensor observation, not the callback receipt time."""
    try:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = now_seconds - stamp
        return age if stamp > 0 and 0 <= age else float('inf')
    except AttributeError:
        return float('inf')


def normalized_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def steering_from_yaw_rate(speed, yaw_rate, wheelbase, max_steer_deg=50.0):
    """Convert left-positive bicycle yaw rate to right-positive steering."""
    if abs(speed) < 1e-6:
        return 0.0
    angle = math.atan(wheelbase * yaw_rate / speed)
    return max(-1.0, min(1.0, -angle / math.radians(max_steer_deg)))


def confirmed_counter(seen, count, active, required=3, release=3):
    """Consecutive confirmation and release, with an explicit active latch."""
    count = max(0, count) + 1 if seen else min(0, count) - 1
    if seen and count >= required:
        active = True
    elif not seen and -count >= release:
        active = False
    return max(-release, min(required, count)), active


def swept_path_clear(scan, speed, steering, offset=math.pi, wheelbase=.14,
                     half_width=.12, front=.22, rear=.22, horizon=.15,
                     steering_max_deg=50., right_boost=1.3):
    """Conservative planar footprint sweep; dimensions include clearance margin.

    Laser origin is assumed at base_link. Physical dimensions/extrinsics must
    be measured. This check never claims that missing returns are free space.
    """
    if abs(speed) < 1e-6:
        return True
    physical_steering = steering*right_boost if steering > 0 else steering
    curvature = -math.tan(max(-1., min(1., physical_steering))*math.radians(steering_max_deg))/wheelbase
    direction = 1 if speed > 0 else -1
    poses = []
    for step in range(11):
        distance = direction*horizon*step/10
        yaw = distance*curvature
        x = math.sin(yaw)/curvature if abs(curvature) > 1e-6 else distance
        y = (1-math.cos(yaw))/curvature if abs(curvature) > 1e-6 else 0.
        poses.append((x,y,math.cos(yaw),math.sin(yaw)))
    directional_returns = 0
    for index, radius in enumerate(scan.ranges):
        if not math.isfinite(radius) or not scan.range_min < radius <= scan.range_max:
            continue
        angle = scan.angle_min+index*scan.angle_increment+offset
        px,py = radius*math.cos(angle), radius*math.sin(angle)
        if direction*px > 0 and abs(py) <= abs(px):
            directional_returns += 1
        for x,y,c,s in poses:
            dx,dy = px-x,py-y
            longitudinal, lateral = dx*c+dy*s, -dx*s+dy*c
            if -rear <= longitudinal <= front and abs(lateral) <= half_width:
                return False
    return directional_returns >= 3
