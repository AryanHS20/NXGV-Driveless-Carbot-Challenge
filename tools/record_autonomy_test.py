#!/usr/bin/env python3
"""Record one operator-controlled autonomy test and stop after AUTO returns to MANUAL.

The recorder is read-only: it never publishes mode, motion, or actuator commands.
It stores full ROS messages in a bag and a compact JSONL stream for quick analysis.
"""

from datetime import datetime, timezone
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


ROOT = Path('/home/sunrise/track_test_run/trials')
SESSION_LIMIT_SEC = 15 * 60
POST_MANUAL_SEC = 3.0
STARTUP_LIMIT_SEC = 20.0

BAG_TOPICS = [
    '/camera/color/image_raw', '/camera/color/camera_info', '/scan', '/odom', '/joy',
    '/imu/rpy', '/auto_mode', '/dashboard_state', '/motion_permitted',
    '/cmd_vel_auto', '/cmd_vel_v4_raw', '/servo_geometry', '/v4_control/status',
    '/cmd_safety_status', '/v4_experimental/bev/primary/image',
    '/v4_experimental/bev/primary/coverage',
    '/v4_experimental/road/primary/candidate',
    '/v4_experimental/road/primary/connected',
    '/v4_experimental/road/primary/fused',
    '/v4_experimental/road/status', '/v4_experimental/trajectory/status',
    '/v4_experimental/arbitration/status',
    '/tunnel_detected', '/tunnel_cmd_vel', '/tunnel_debug',
]

STATUS_TOPICS = (
    '/dashboard_state', '/cmd_safety_status', '/v4_control/status',
    '/v4_experimental/road/status', '/v4_experimental/trajectory/status',
    '/v4_experimental/arbitration/status', '/imu/rpy',
)

MASK_TOPICS = (
    '/v4_experimental/bev/primary/coverage',
    '/v4_experimental/road/primary/candidate',
    '/v4_experimental/road/primary/connected',
    '/v4_experimental/road/primary/fused',
)


def compact_status(topic, data):
    if not isinstance(data, dict):
        return data
    if topic.endswith('/trajectory/status'):
        return {key: data.get(key) for key in (
            'selected_diagnostic_only', 'blockers', 'processed_frames',
            'last_error', 'last_warning',
        )}
    if topic.endswith('/road/status'):
        return {key: data.get(key) for key in (
            'profiles', 'corridor', 'processed_frames', 'last_error',
        )}
    return data


def image_metrics(msg):
    """Return small mask metrics without requiring cv_bridge or NumPy."""
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width or not msg.data:
        return {'width': width, 'height': height, 'nonzero_fraction': 0.0}
    total = width * height
    nonzero = 0
    sum_x = 0
    sum_y = 0
    view = memoryview(msg.data)
    for y in range(height):
        row = view[y * step:y * step + width]
        for x, value in enumerate(row):
            if value:
                nonzero += 1
                sum_x += x
                sum_y += y
    result = {
        'width': width, 'height': height,
        'nonzero_fraction': round(nonzero / total, 5),
    }
    if nonzero:
        result['centroid_x_px'] = round(sum_x / nonzero, 2)
        result['centroid_y_px'] = round(sum_y / nonzero, 2)
    return result


def summarize(events, intervals, counts, duration, ready, latest, directory):
    commands = [event for event in events if event['topic'] == '/cmd_vel_auto']
    auto_commands = []
    for event in commands:
        if any(item['start_t'] <= event['t'] <= item['end_t'] for item in intervals):
            auto_commands.append(event['data'])
    duties = [abs(float(item['linear_x'])) * 255.0 for item in auto_commands]
    steer = [float(item['steer_right_normalized']) for item in auto_commands]
    trajectory = [event['data'] for event in events
                  if event['topic'].endswith('/trajectory/status')]
    selected = [item.get('selected_diagnostic_only') for item in trajectory
                if isinstance(item, dict) and item.get('selected_diagnostic_only')]
    low_support_holds = sum(
        item.get('steering_source') == 'low_support_direction_hold'
        for item in selected
    )
    recoveries = sum(
        isinstance(event['data'], dict)
        and event['data'].get('boundary_recovery_active') is True
        for event in events if event['topic'] == '/v4_control/status'
    )
    return {
        'directory': str(directory), 'duration_sec': round(duration, 3),
        'ready': ready, 'auto_observed': bool(intervals),
        'auto_intervals': intervals, 'last_mode': latest.get('/auto_mode'),
        'last_final_command': latest.get('/cmd_vel_auto'), 'counts': counts,
        'analysis': {
            'auto_command_samples': len(auto_commands),
            'motor_duty_percent': {
                'minimum': round(min(duties), 2) if duties else None,
                'mean': round(sum(duties) / len(duties), 2) if duties else None,
                'maximum': round(max(duties), 2) if duties else None,
            },
            'steer_right_normalized': {
                'minimum': round(min(steer), 4) if steer else None,
                'mean': round(sum(steer) / len(steer), 4) if steer else None,
                'maximum': round(max(steer), 4) if steer else None,
            },
            'zero_command_fraction': round(
                sum(duty < 0.5 for duty in duties) / len(duties), 4
            ) if duties else None,
            'low_support_direction_holds': low_support_holds,
            'boundary_reverse_status_samples': recoveries,
        },
        'motion_or_mode_requests_published': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--continuous', action='store_true',
        help='keep recording across MANUAL repositioning until Ctrl-C',
    )
    args = parser.parse_args()
    if subprocess.check_output(['hostname'], text=True).strip() not in ('risabot1', 'risabot5'):
        raise RuntimeError('this recorder must run on risabot1 or risabot5')
    directory = ROOT / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory.mkdir(parents=True)
    log = (directory / 'bag.log').open('w')
    bag = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(directory / 'bag'), *BAG_TOPICS],
        stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    rclpy.init()
    node = Node('autonomy_test_read_only_recorder')
    start = time.monotonic()
    latest, counts, events, intervals = {}, {}, [], []
    mask_metric_stamps = {}
    active_since = None
    previous_mode = None
    ready = False
    after_manual_deadline = None
    stream = (directory / 'events.jsonl').open('w', buffering=1)

    def record(topic, data):
        latest[topic] = data
        counts[topic] = counts.get(topic, 0) + 1
        event = {'t': round(time.monotonic() - start, 6), 'topic': topic, 'data': data}
        events.append(event)
        stream.write(json.dumps(event, separators=(',', ':')) + '\n')

    def status(topic, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = msg.data
        record(topic, compact_status(topic, data))

    def mask(topic, msg):
        now = time.monotonic()
        if now - mask_metric_stamps.get(topic, 0.0) < 0.5:
            return
        mask_metric_stamps[topic] = now
        record(topic, image_metrics(msg))

    def mode(msg):
        nonlocal active_since, previous_mode, after_manual_deadline
        enabled = bool(msg.data)
        record('/auto_mode', enabled)
        now = time.monotonic()
        if enabled and active_since is None:
            active_since = now
            after_manual_deadline = None
        elif not enabled and active_since is not None:
            intervals.append({
                'start_t': round(active_since - start, 6),
                'end_t': round(now - start, 6),
                'duration_sec': round(now - active_since, 6),
            })
            active_since = None
            after_manual_deadline = now + POST_MANUAL_SEC
        if previous_mode is not enabled:
            print(json.dumps({'mode': 'AUTO' if enabled else 'MANUAL',
                              't': round(now - start, 3)}), flush=True)
            previous_mode = enabled

    node.create_subscription(Bool, '/auto_mode', mode, 10)
    for topic in STATUS_TOPICS:
        node.create_subscription(String, topic, lambda msg, key=topic: status(key, msg), 10)
    for topic in ('/cmd_vel_auto', '/cmd_vel_v4_raw'):
        node.create_subscription(Twist, topic, lambda msg, key=topic: record(key, {
            'linear_x': msg.linear.x, 'steer_right_normalized': msg.angular.z,
        }), 10)
    node.create_subscription(Odometry, '/odom', lambda msg: record('/odom', {
        'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
        'speed': msg.twist.twist.linear.x, 'yaw_rate': msg.twist.twist.angular.z,
    }), 10)
    for topic in MASK_TOPICS:
        node.create_subscription(
            Image, topic, lambda msg, key=topic: mask(key, msg), 2
        )

    try:
        while time.monotonic() - start < SESSION_LIMIT_SEC:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not ready and latest.get('/auto_mode') is False and counts.get('/cmd_vel_auto', 0) > 10:
                log.flush()
                bag_text = (directory / 'bag.log').read_text(errors='replace')
                if bag.poll() is not None:
                    raise RuntimeError('rosbag recorder exited: ' + bag_text[-1200:])
                if 'Recording' in bag_text:
                    ready = True
                    print(json.dumps({
                        'recording_ready': True, 'directory': str(directory),
                        'operator_selects_auto': True,
                        'stops_after_manual': not args.continuous,
                    }), flush=True)
            if not ready and time.monotonic() - start > STARTUP_LIMIT_SEC:
                raise RuntimeError(
                    'recorder did not receive MANUAL and drive commands; '
                    'check ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY before testing'
                )
            if (after_manual_deadline is not None
                    and time.monotonic() >= after_manual_deadline
                    and latest.get('/auto_mode') is False
                    and not args.continuous):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if active_since is not None:
            now = time.monotonic()
            intervals.append({'start_t': round(active_since - start, 6),
                              'end_t': round(now - start, 6),
                              'duration_sec': round(now - active_since, 6),
                              'ended_by_recorder': True})
        stream.flush()
        stream.close()
        if bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=12)
            except subprocess.TimeoutExpired:
                bag.kill()
                bag.wait(timeout=3)
        log.close()
        result = summarize(events, intervals, counts, time.monotonic() - start,
                           ready, latest, directory)
        (directory / 'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
