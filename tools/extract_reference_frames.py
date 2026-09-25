#!/usr/bin/env python3
"""Extract synchronized images from a recorded Risabot 1 perception frame.

Run on the ROS 2 host with BAG_DB EVENTS_JSONL OUTPUT_DIR EVENT_TIME [...].
Match each event's corridor to the full bag status, then use its source image
timestamp to retrieve the synchronized camera, BEV, coverage, and masks.
"""

import argparse
import json
import sqlite3
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String


ROAD = '/v4_experimental/road/status'
IMAGES = {
    'raw': '/camera/color/image_raw',
    'bev': '/v4_experimental/bev/primary/image',
    'coverage': '/v4_experimental/bev/primary/coverage',
    'candidate': '/v4_experimental/road/primary/candidate',
    'connected': '/v4_experimental/road/primary/connected',
    'fused': '/v4_experimental/road/primary/fused',
}


def corridor_signature(payload):
    corridor = payload.get('corridor', {})
    return json.dumps(corridor.get('primary', []) if isinstance(corridor, dict)
                      else [], sort_keys=True, separators=(',', ':'))


def image_array(message):
    raw = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    encoding = message.encoding.lower()
    if encoding in ('rgb8', 'bgr8'):
        image = raw[:, :message.width * 3].reshape(message.height, message.width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else image
    if encoding == 'mono8':
        return raw[:, :message.width]
    raise ValueError(f'unsupported image encoding: {message.encoding}')


def image_stamp(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_db', type=Path)
    parser.add_argument('events_jsonl', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('event_time', nargs='+', type=float)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    road_events = []
    for line in args.events_jsonl.open(encoding='utf-8'):
        event = json.loads(line)
        if event.get('topic') == ROAD and isinstance(event.get('data'), dict):
            road_events.append(event)
    if not road_events:
        raise RuntimeError('no road frame counters in event log')

    with sqlite3.connect(f'file:{args.bag_db}?mode=ro', uri=True) as db:
        topic_ids = {name: identity for identity, name in db.execute(
            'SELECT id, name FROM topics')}
        if ROAD not in topic_ids:
            raise RuntimeError('bag has no road status topic')
        selected_events = [min(road_events, key=lambda event: abs(event['t'] - at))
                           for at in args.event_time]
        wanted = {corridor_signature(event['data']) for event in selected_events}
        statuses = {signature: [] for signature in wanted}
        for timestamp, blob in db.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=?',
                (topic_ids[ROAD],)):
            payload = json.loads(deserialize_message(blob, String).data)
            signature = corridor_signature(payload)
            if signature in wanted:
                statuses[signature].append((timestamp, payload))

        anchor_event = next((event for event in selected_events
                             if statuses[corridor_signature(event['data'])]), None)
        if anchor_event is None:
            raise RuntimeError('selected event corridors are absent from bag')
        anchor_candidates = statuses[corridor_signature(anchor_event['data'])]
        anchor_stamp = anchor_candidates[0][0]
        offset = anchor_stamp - int(anchor_event['t'] * 1e9)

        for at in args.event_time:
            event = min(road_events, key=lambda item: abs(item['t'] - at))
            matches = statuses[corridor_signature(event['data'])]
            if not matches:
                print(f'{at:g}s: road corridor is absent from bag')
                continue
            road_timestamp, status = min(
                matches, key=lambda item: abs(item[0] - offset - int(event['t'] * 1e9)))
            count = int(status['frames_published']['primary'])
            source_stamp = float(status['last_image_stamp_sec']['primary'])
            folder = args.output_dir / f'{at:g}s_frame_{count}'
            folder.mkdir(parents=True, exist_ok=True)
            manifest = {
                'event_time_sec': at,
                'matched_event_time_sec': event['t'],
                'road_frame': count,
                'source_stamp_sec': source_stamp,
                'road_status': status,
                'images': {},
            }
            for label, topic in IMAGES.items():
                if topic not in topic_ids:
                    continue
                start = road_timestamp - int(1.0e9)
                end = road_timestamp + int(0.3e9)
                nearest = None
                for (blob,) in db.execute(
                        'SELECT data FROM messages WHERE topic_id=? '
                        'AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                        (topic_ids[topic], start, end)):
                    message = deserialize_message(blob, Image)
                    difference = abs(image_stamp(message) - source_stamp)
                    if nearest is None or difference < nearest[0]:
                        nearest = (difference, message)
                if nearest is None:
                    continue
                difference, message = nearest
                if difference > 0.05:
                    print(f'{at:g}s {label}: nearest image differs by {difference:.3f}s')
                    continue
                output = folder / f'{label}.png'
                if not cv2.imwrite(str(output), image_array(message)):
                    raise RuntimeError(f'could not write {output}')
                manifest['images'][label] = {
                    'source_stamp_difference_sec': difference,
                    'path': output.name,
                }
            (folder / 'manifest.json').write_text(
                json.dumps(manifest, indent=2), encoding='utf-8')
            print(f'{at:g}s: frame {count}, {len(manifest["images"])} images')


if __name__ == '__main__':
    main()
