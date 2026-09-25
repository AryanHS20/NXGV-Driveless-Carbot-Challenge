#!/usr/bin/env python3
"""Render synchronized camera, BEV mask, and chosen lane from one ROS trial.

Run on a ROS 2 host with: render_trial_perception.py BAG EVENTS OUTPUT T1 [T2 ...]
Times are seconds in the read-only recorder's events.jsonl.
"""

import json
import statistics
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


RAW = '/camera/color/image_raw'
BEV = '/v4_experimental/bev/primary/image'
MASK = '/v4_experimental/road/primary/connected'
CANDIDATE = '/v4_experimental/road/primary/candidate'
MODE = '/auto_mode'


def as_bgr(message):
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    name = message.encoding.lower()
    if name in ('bgr8', 'rgb8'):
        frame = raw[:, :message.width * 3].reshape(message.height, message.width, 3)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if name == 'rgb8' else frame.copy()
    if name == 'mono8':
        return cv2.cvtColor(raw[:, :message.width], cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported image encoding {message.encoding}')


def nearest_event(events, topic, at):
    matches = (event for event in events if event['topic'] == topic)
    return min(matches, key=lambda event: abs(event['t'] - at), default=None)


def latest_event(events, topic, at):
    return next((event for event in reversed(events)
                 if event['topic'] == topic and event['t'] <= at), None)


def canvas_line(image, text, y, color=(240, 240, 240)):
    cv2.putText(image, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.54, color, 1, cv2.LINE_AA)


def draw_bev(image, road):
    """Draw the metric lane reference actually used by the controller."""
    if road is None or not isinstance(road.get('data'), dict):
        return image
    samples = road['data'].get('corridor', {}).get('primary', [])
    height, width = image.shape[:2]

    def pixel(forward, left):
        # R1 calibrated profile: left [-.90,.90], forward [.05,1.80] m.
        u = round((.90 - left) * (width - 1) / 1.80)
        v = round((1.80 - forward) * (height - 1) / 1.75)
        return (u, v)

    overlay = image.copy()
    for sample in samples:
        x = float(sample['forward_m'])
        if not .05 <= x <= 1.80:
            continue
        center = float(sample['left_m'])
        left_seen = sample.get('left_boundary_observed', False)
        right_seen = sample.get('right_boundary_observed', False)
        if left_seen and right_seen:
            left = float(sample.get('left_boundary_m', center + .5 * sample['width_m']))
            right = float(sample.get('right_boundary_m', center - .5 * sample['width_m']))
            target = .5 * (left + right)
        elif left_seen:
            left = float(sample.get('left_boundary_m', center + .5 * sample['width_m']))
            target = left - .1475
        elif right_seen:
            right = float(sample.get('right_boundary_m', center - .5 * sample['width_m']))
            target = right + .1475
        else:
            continue
        cv2.circle(overlay, pixel(x, target), 3, (0, 220, 255), -1)
        if left_seen:
            cv2.circle(overlay, pixel(x, left), 2, (50, 50, 255), -1)
        if right_seen:
            cv2.circle(overlay, pixel(x, right), 2, (255, 100, 30), -1)
    # Vehicle midpoint on the nearest visible row. BEV is blind below x=.05.
    cv2.line(overlay, pixel(.05, 0), pixel(.30, 0), (255, 255, 0), 2)
    return overlay


def main():
    bag_path, events_path, output = sys.argv[1:4]
    targets = [float(value) for value in sys.argv[4:]]
    if not targets:
        raise SystemExit('provide one or more event times')
    events = [json.loads(line) for line in Path(events_path).open(encoding='utf-8')]
    event_transitions = []
    for event in events:
        if event['topic'] == MODE and (not event_transitions or
                                      event['data'] != event_transitions[-1]['data']):
            event_transitions.append(event)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                           output_serialization_format='cdr'))
    bag_transitions = []
    frames = {name: [] for name in (RAW, BEV, MASK, CANDIDATE)}
    previous_mode = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == MODE:
            enabled = bool(deserialize_message(data, Bool).data)
            if enabled != previous_mode:
                bag_transitions.append((stamp / 1e9, enabled))
                previous_mode = enabled
        elif topic in frames:
            frames[topic].append((stamp / 1e9, data))
    matches = [(b, e) for (b, state), e in zip(bag_transitions, event_transitions)
               if state == e['data']]
    if not matches:
        raise RuntimeError('cannot align recorder events with bag mode transitions')
    print('bag mode transitions', [(round(stamp, 3), state)
                                   for stamp, state in bag_transitions])
    print('event mode transitions', [(event['t'], event['data'])
                                     for event in event_transitions])
    offsets = [b - e['t'] for b, e in matches]
    # The bag recorder can attach after the first mode change. Later mode
    # changes come from the same two active subscribers and align tightly.
    if len(offsets) > 1 and abs(offsets[0] - statistics.median(offsets[1:])) > .5:
        offsets.pop(0)
    offset = statistics.median(offsets)
    print('bag/event alignment offsets', [round(value, 3) for value in offsets])
    rows = []
    for target in targets:
        images = {}
        for topic, items in frames.items():
            if not items:
                raise RuntimeError(f'{topic} has no bag frames')
            stamp, data = min(items, key=lambda item: abs(item[0] - (offset + target)))
            images[topic] = as_bgr(deserialize_message(data, Image))
            print(f't={target:.2f} {topic} delta={stamp-offset-target:+.3f}s')
        road = nearest_event(events, '/v4_experimental/road/status', target)
        trajectory = nearest_event(events, '/v4_experimental/trajectory/status', target)
        mode = latest_event(events, MODE, target)
        auto_enabled = bool(mode and mode['data'])
        command = nearest_event(events, '/cmd_vel_auto' if auto_enabled else '/cmd_vel', target)
        mask = images[MASK]
        candidate = images[CANDIDATE]
        bev = images[BEV]
        tint = bev.copy()
        positive = mask[:, :, 0] > 0
        discarded = (candidate[:, :, 0] > 0) & ~positive
        tint[discarded] = (.35 * tint[discarded] +
                           .65 * np.array([185, 40, 220])).astype(np.uint8)
        tint[positive] = (.35 * tint[positive] + .65 * np.array([40, 175, 40])).astype(np.uint8)
        tint = draw_bev(tint, road)
        raw = cv2.resize(images[RAW], (640, 480), interpolation=cv2.INTER_LINEAR)
        tint = cv2.resize(tint, (640, 480), interpolation=cv2.INTER_NEAREST)
        row = np.full((584, 1300, 3), (24, 30, 36), np.uint8)
        row[38:518, :640] = raw
        row[38:518, 660:] = tint
        selected = ((trajectory or {}).get('data') or {}).get('selected_diagnostic_only') or {}
        cmd = (command or {}).get('data') or {}
        canvas_line(row, f'RAW CAMERA  |  {target:.2f} s', 27)
        cv2.putText(row, 'CALIBRATED BEV + CONNECTED ROAD MASK', (674, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, .54, (100, 220, 255), 1, cv2.LINE_AA)
        canvas_line(row, 'GREEN selected road  |  PINK rejected dark road  |  YELLOW target  |  RED/BLUE white edges', 541)
        motion = (f"AUTO motor={round(float(cmd.get('linear_x', 0)) * 255)}%"
                  if auto_enabled else
                  f"MANUAL stick throttle={float(cmd.get('linear_x', 0)):.2f}")
        canvas_line(row, f"Planner suggestion: steer={selected.get('command_steer_rad_diagnostic_only')} rad; "
                         f"clearance={selected.get('boundary_clearance_m')} m; {motion}", 568,
                    (100, 220, 255))
        rows.append(row)
    result = np.vstack(rows)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(output, result):
        raise RuntimeError(f'failed to write {output}')
    print('saved', output)


if __name__ == '__main__':
    main()
