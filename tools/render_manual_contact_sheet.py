#!/usr/bin/env python3
"""Make a timed contact sheet from a manual-reference ROS 2 bag."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def main():
    bag_path, events_path, output = sys.argv[1:4]
    targets = [float(value) for value in sys.argv[4:]]
    if not targets:
        raise SystemExit('provide event times')
    events = [json.loads(line) for line in Path(events_path).open(encoding='utf-8')]
    event_transitions = []
    for event in events:
        if event['topic'] == '/auto_mode' and (not event_transitions or
             event['data'] != event_transitions[-1]['data']):
            event_transitions.append(event)
    first = event_transitions[0]
    last = event_transitions[-1] if len(event_transitions) > 1 else None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                           output_serialization_format='cdr'))
    transitions = []
    images = []
    previous = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/auto_mode':
            mode = bool(deserialize_message(data, Bool).data)
            if mode != previous:
                transitions.append((stamp / 1e9, mode))
                previous = mode
        elif topic == '/camera/color/image_raw':
            images.append((stamp / 1e9, data))
    if not images or not transitions:
        raise RuntimeError('camera or mode data missing')
    # The final manual->auto transition, when present, anchors the same
    # monotonic event clock after ROS bag startup delays have settled.
    if last and transitions[-1][1] == last['data']:
        offset = transitions[-1][0] - last['t']
    else:
        offset = transitions[0][0] - first['t']
    tiles = []
    for at in targets:
        stamp, data = min(images, key=lambda item: abs(item[0] - offset - at))
        msg = deserialize_message(data, Image)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        if msg.encoding.lower() == 'rgb8':
            frame = cv2.cvtColor(raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3),
                                 cv2.COLOR_RGB2BGR)
        elif msg.encoding.lower() == 'bgr8':
            frame = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        else:
            raise ValueError(msg.encoding)
        tile = np.full((278, 336, 3), (25, 30, 35), np.uint8)
        tile[32:272, 8:328] = cv2.resize(frame, (320, 240))
        cv2.putText(tile, f'{at:.0f}s  (delta {stamp-offset-at:+.2f}s)', (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 230, 130), 1, cv2.LINE_AA)
        tiles.append(tile)
    columns = 4
    blank = np.full_like(tiles[0], (25, 30, 35))
    rows = [np.hstack((tiles[i:i+columns] + [blank] * columns)[:columns])
            for i in range(0, len(tiles), columns)]
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(output, np.vstack(rows))
    print('saved', output, 'offset', offset)


if __name__ == '__main__':
    main()
