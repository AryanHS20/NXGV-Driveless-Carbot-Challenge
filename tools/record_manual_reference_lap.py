#!/usr/bin/env python3
"""Record a human-driven reference lap until interrupted by the operator."""

from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool, Float32, String

from record_autonomy_test import compact_status, image_metrics


ROOT = Path('/home/sunrise/track_test_run/manual_reference_laps')
SESSION_LIMIT_SEC = 20 * 60
BAG_TOPICS = [
    '/camera/color/image_raw', '/camera/color/camera_info', '/scan', '/odom',
    '/joy', '/auto_mode', '/dashboard_state', '/motion_permitted', '/cmd_vel',
    '/cmd_vel_auto', '/cmd_vel_v4_raw', '/servo_geometry', '/v4_control/status',
    '/cmd_safety_status', '/imu/rpy', '/fused_heading', '/lane_curvature',
    '/v4_experimental/bev/primary/image',
    '/v4_experimental/bev/primary/coverage',
    '/v4_experimental/road/primary/candidate',
    '/v4_experimental/road/primary/connected',
    '/v4_experimental/road/primary/fused',
    '/v4_experimental/road/status', '/v4_experimental/trajectory/status',
    '/v4_experimental/arbitration/status',
]
MASK_TOPICS = (
    '/v4_experimental/bev/primary/coverage',
    '/v4_experimental/road/primary/candidate',
    '/v4_experimental/road/primary/connected',
    '/v4_experimental/road/primary/fused',
)
STATUS_TOPICS = (
    '/dashboard_state', '/cmd_safety_status', '/v4_control/status',
    '/v4_experimental/road/status', '/v4_experimental/trajectory/status',
    '/v4_experimental/arbitration/status', '/imu/rpy', '/servo_geometry',
)


def main():
    hostname = subprocess.check_output(['hostname'], text=True).strip()
    if hostname not in ('risabot1', 'risabot5'):
        raise RuntimeError('this recorder must run on risabot1 or risabot5')
    directory = ROOT / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory.mkdir(parents=True)
    bag_log = (directory / 'bag.log').open('w')
    bag = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(directory / 'bag'), *BAG_TOPICS],
        stdout=bag_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    rclpy.init()
    node = Node('manual_reference_lap_read_only_recorder')
    start = time.monotonic()
    latest, counts, events, mask_stamps = {}, {}, [], {}
    ready = False
    stream = (directory / 'events.jsonl').open('w')

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
        if now - mask_stamps.get(topic, 0.0) < 0.5:
            return
        mask_stamps[topic] = now
        record(topic, image_metrics(msg))

    node.create_subscription(Bool, '/auto_mode', lambda msg: record(
        '/auto_mode', bool(msg.data)), 10)
    for topic in STATUS_TOPICS:
        node.create_subscription(String, topic, lambda msg, key=topic: status(key, msg), 10)
    for topic in ('/cmd_vel', '/cmd_vel_auto', '/cmd_vel_v4_raw'):
        node.create_subscription(Twist, topic, lambda msg, key=topic: record(key, {
            'linear_x': msg.linear.x, 'steer_right_normalized': msg.angular.z,
        }), 10)
    node.create_subscription(Odometry, '/odom', lambda msg: record('/odom', {
        'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
        'speed': msg.twist.twist.linear.x, 'yaw_rate': msg.twist.twist.angular.z,
    }), 10)
    node.create_subscription(Joy, '/joy', lambda msg: record('/joy', {
        'axes': list(msg.axes), 'buttons': list(msg.buttons),
    }), 10)
    node.create_subscription(Float32, '/fused_heading', lambda msg: record(
        '/fused_heading', float(msg.data)), 10)
    node.create_subscription(Float32, '/lane_curvature', lambda msg: record(
        '/lane_curvature', float(msg.data)), 10)
    for topic in MASK_TOPICS:
        node.create_subscription(Image, topic, lambda msg, key=topic: mask(key, msg), 2)

    interrupted = False
    try:
        while time.monotonic() - start < SESSION_LIMIT_SEC:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not ready and latest.get('/auto_mode') is False and counts.get('/joy', 0) > 10:
                bag_log.flush()
                bag_text = (directory / 'bag.log').read_text(errors='replace')
                if bag.poll() is not None:
                    raise RuntimeError('rosbag recorder exited: ' + bag_text[-1200:])
                if 'Recording' in bag_text:
                    ready = True
                    print(json.dumps({
                        'recording_ready': True, 'directory': str(directory),
                        'drive_mode': 'MANUAL', 'stop_instruction': 'tell Codex lap done',
                    }), flush=True)
    except KeyboardInterrupt:
        interrupted = True
        print(json.dumps({'stop_requested': True}), flush=True)
    finally:
        stream.flush()
        stream.close()
        if bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag.kill()
                bag.wait(timeout=3)
        bag_log.close()
        result = {
            'directory': str(directory),
            'duration_sec': round(time.monotonic() - start, 3),
            'ready': ready, 'manual_mode_preserved': latest.get('/auto_mode') is False,
            'operator_stop_received': interrupted, 'counts': counts,
            'last_applied_command': latest.get('/cmd_vel'),
            'motion_or_mode_requests_published': False,
        }
        (directory / 'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
