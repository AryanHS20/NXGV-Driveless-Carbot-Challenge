"""Mission selection and independent stop constraints.

No distance threshold is evidence of a roundabout exit. A route needs a gate
observation, an observed lane branch, and departure from the roundabout sign.
"""
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .control_contract import fresh


def reset_mission(driver):
    driver.mission_finished = False
    driver.mission_fault = ''
    driver.red_latched = False
    driver.route = ''
    driver.route_confirmed = ''
    driver.roundabout_seen = False
    driver.roundabout_active = False
    driver.roundabout_stamp = 0.0
    driver.route_stamp = 0.0
    driver.parking_kind = ''
    driver.parking_kind_stamp = 0.0
    driver.parking_requested = ''
    driver.parking_acknowledged = False
    driver.parking_request_time = 0.0
    driver.parking_result = ''
    driver.parking_result_kind = ''
    driver.parking_result_stamp = 0.0
    driver.completed_parking = set()
    driver.tunnel_was_seen = False
    driver.light_cleared = False
    driver.light_expected = False


def select_command(d, now):
    # Import here to avoid a module initialization cycle.
    from .auto_driver import ChallengeState as S
    zero = Twist()
    ttl = float(d._param_cache['stale_timeout'])
    def recent(stamp): return fresh(stamp, now, ttl)
    if d.mission_finished:
        return S.FINISHED, zero, 'MISSION COMPLETE'

    if recent(d.roundabout_stamp) and d.roundabout_active:
        d.roundabout_seen = True
    if d.roundabout_seen and not d.route_confirmed and recent(d.boom_gate_last_time):
        d.route = 'through' if d.boom_gate_open else 'right'
    if d.route:
        d.route_pub.publish(String(data=d.route))

    # Remember a red observation even when a different behavior is active.
    if recent(d.traffic_light_last_time):
        if d.traffic_light_state in ('red', 'yellow'):
            d.red_latched = True
            d.light_cleared = False
        elif d.traffic_light_state == 'green':
            d.red_latched = False
            d.light_cleared = True

    stop = ''
    if d.cmd_safety_estop:
        stop = 'E-STOP ACTIVE'
    elif not d.motion_permitted or not recent(d.permit_stamp):
        stop = 'SENSOR INTERLOCK'
    elif d.red_latched:
        stop = 'AWAITING CONFIRMED GREEN'
    elif d.light_expected and not d.light_cleared:
        stop = 'WAITING FOR TRAFFIC LIGHT CLEARANCE'
    elif recent(d.boom_gate_last_time) and not d.boom_gate_open and not (
            d.route == 'right' and d.route_confirmed == 'right' and recent(d.route_stamp)
            and recent(d.lane_stamp) and not d.lane_lost and d.lane_error > .05):
        # Only an observed right branch may divert around a closed gate.
        # The final safety node independently checks the commanded swept path.
        stop = 'BOOM GATE CLOSED'
    if stop:
        if d.parking_requested:
            d.rp_cmd_pub.publish(String(data='stop'))
            d.mission_fault = 'Parking interrupted: ' + stop
        state = S.TRAFFIC_LIGHT if d.red_latched else S.EMERGENCY_STOP
        return state, zero, stop
    if d.mission_finished:
        return S.FINISHED, zero, 'MISSION COMPLETE'
    if d.mission_fault:
        return S.EMERGENCY_STOP, zero, d.mission_fault

    # Every moving behavior needs current observations, not heartbeat receipt.
    if not all(recent(stamp) for stamp in (d.obstruction_last_time, d.tunnel_last_time, d.boom_gate_last_time)):
        return S.EMERGENCY_STOP, zero, 'WAITING FOR PERCEPTION'

    if d.parking_requested:
        kind = d.parking_requested
        if recent(d.parking_result_stamp) and d.parking_result_kind == kind and d.parking_result_stamp >= d.parking_request_time:
            if d.rp_state == 'PLAYBACK' and d.parking_result == 'running':
                d.parking_acknowledged = True
            elif d.parking_result == 'complete' and d.parking_acknowledged:
                d.completed_parking.add(kind)
                d.parking_requested = ''
                d.parking_sequence_active = False
                if kind == 'perpendicular':
                    d.mission_finished = True
                    return S.FINISHED, zero, 'MISSION COMPLETE'
                return S.LANE_FOLLOW, zero, 'PARALLEL COMPLETE'
            elif d.parking_result not in ('', 'running'):
                d.mission_fault = 'Parking failed: ' + d.parking_result
        if not d.parking_acknowledged and now - d.parking_request_time > 2.0:
            d.mission_fault = 'Parking start not acknowledged'
        if now - d.parking_request_time > float(d._param_cache['parking_max_duration']):
            d.mission_fault = 'Parking completion timeout'
        if d.mission_fault:
            d.rp_cmd_pub.publish(String(data='stop'))
            return S.EMERGENCY_STOP, zero, d.mission_fault
        if d.parking_acknowledged and not recent(d.parking_result_stamp):
            d.mission_fault = 'Parking feedback lost'
            d.rp_cmd_pub.publish(String(data='stop'))
            return S.EMERGENCY_STOP, zero, d.mission_fault
        return S.PARKING_PLAYBACK, zero, 'PARKING ' + kind.upper()

    # Gate selection is retained through the junction; distance does not choose
    # an exit. The lane node confirms only when it sees distinct lane candidates.
    if recent(d.roundabout_stamp) and d.roundabout_active:
        d.roundabout_seen = True
    if d.roundabout_seen:
        if not recent(d.lane_stamp) or not recent(d.lane_lost_stamp) or d.lane_lost:
            return S.LANE_RECOVERY, zero, "LANE MISSING AT JUNCTION"
        if not d.route:
            if not recent(d.boom_gate_last_time):
                return S.ROUNDABOUT, zero, 'WAITING FOR GATE OBSERVATION'
            d.route = 'through' if d.boom_gate_open else 'right'
        d.route_pub.publish(String(data=d.route))
        if (recent(d.route_stamp) and d.route_confirmed == d.route
                and recent(d.roundabout_stamp) and not d.roundabout_active):
            if d.route == 'right':
                d.set_parameters([rclpy.Parameter('current_lap', rclpy.Parameter.Type.INTEGER, 2)])
            d._boom_gate_armed = True
            d.roundabout_seen = False
            d.route = ''
            d.route_confirmed = ''
            d.route_pub.publish(String(data='follow'))
        else:
            return S.ROUNDABOUT, d._lane_follow_cmd(), 'ROUTE ' + d.route.upper()

    # A proximity flag is a stop, never an unobserved reverse maneuver.
    if d.tunnel_detected:
        if not recent(d.tunnel_cmd_stamp):
            return S.EMERGENCY_STOP, zero, 'TUNNEL COMMAND STALE'
        d.tunnel_was_seen = True
        return S.TUNNEL, d.tunnel_cmd, ''
    if d.obstacle_active and not d.obstruction_active:
        return S.EMERGENCY_STOP, zero, 'OBSTACLE: STOPPED'
    if d.obstacle_sign_active and recent(d.obstacle_sign_stamp) and not d.obstruction_active:
        return S.EMERGENCY_STOP, zero, 'OBSTACLE SIGN: WAITING FOR LIDAR PATH'
    if d.obstruction_active:
        if not recent(d.obstruction_cmd_stamp):
            return S.EMERGENCY_STOP, zero, 'OBSTRUCTION COMMAND STALE'
        return S.OBSTRUCTION, d.obstruction_cmd, ''
    if d.tunnel_was_seen:
        d._tl_armed = True

    if not recent(d.lane_stamp) or not recent(d.lane_lost_stamp) or d.lane_lost:
        return S.LANE_RECOVERY, zero, 'LANE MISSING OR STALE'

    if d.current_lap == 2 and recent(d.parking_kind_stamp):
        kind = d.parking_kind
        expected = 'perpendicular' if 'parallel' in d.completed_parking else 'parallel'
        if kind and kind not in d.completed_parking:
            if kind != expected:
                return S.PARKING_IDLE, zero, 'WAITING FOR ' + expected.upper() + ' PARKING'
            if d.state != S.PARKING_IDLE:
                return S.PARKING_IDLE, zero, 'ALIGN PARKING'
            if now - d.state_entry_time >= d._param_cache['parking_idle_duration']:
                d.parking_requested = kind
                d.parking_sequence_active = True
                d.parking_acknowledged = False
                d.parking_request_time = now
                d.parking_result = ''
                d.rp_cmd_pub.publish(String(data='playback:' + kind))
                return S.PARKING_PLAYBACK, zero, 'START PARKING'
            return S.PARKING_IDLE, zero, 'ALIGN PARKING'

    cmd = d._lane_follow_cmd()
    if recent(d.pitch_stamp) and d.current_pitch >= d._param_cache['hill_pitch_threshold']:
        extra = d.current_pitch - d._param_cache['hill_pitch_threshold']
        cmd.linear.x = min(d._param_cache['hill_max_speed'],
                           d._param_cache['hill_base_speed'] + extra * d._param_cache['hill_speed_per_degree'])
        # Retain lane steering; do not arbitrarily suppress it on a hill.
        return S.HILL, cmd, ''
    return S.LANE_FOLLOW, cmd, ''
