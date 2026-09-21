#!/usr/bin/env python3
"""Verify that a recorded track session contains usable critical evidence."""

import argparse
import csv
import json
import os
import sys

import yaml


CRITICAL_TOPICS = (
    '/camera/color/image_raw',
    '/scan',
    '/odom',
)
CORE_V4_TOPICS = (
    '/v4_experimental/bev/status',
    '/v4_experimental/road/status',
    '/v4_experimental/trajectory/status',
    '/v4_experimental/arbitration/status',
)


def message_counts(metadata):
    """Return {topic name: recorded message count} from rosbag2 metadata."""
    info = metadata.get('rosbag2_bagfile_information', {})
    result = {}
    for item in info.get('topics_with_message_count', []):
        try:
            name = str(item['topic_metadata']['name'])
            result[name] = int(item['message_count'])
        except (KeyError, TypeError, ValueError):
            continue
    return result


def read_events(path):
    events = []
    if not os.path.isfile(path):
        return events
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except (TypeError, ValueError):
                continue
            if isinstance(record, dict) and record.get('event'):
                events.append(str(record['event']))
    return events


def max_temperature(path):
    values = []
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8', newline='') as handle:
        for row in csv.DictReader(handle):
            try:
                values.append(float(row['temperature_c']))
            except (KeyError, TypeError, ValueError):
                continue
    return max(values) if values else None


def verify(session_dir):
    metadata_path = os.path.join(session_dir, 'bag', 'metadata.yaml')
    if not os.path.isfile(metadata_path):
        return {
            'ok': False,
            'errors': ['bag/metadata.yaml is missing'],
            'warnings': [],
            'topic_counts': {},
            'events': [],
            'max_temperature_c': None,
        }
    with open(metadata_path, encoding='utf-8') as handle:
        metadata = yaml.safe_load(handle) or {}
    counts = message_counts(metadata)
    errors = [f'{topic} has no recorded messages'
              for topic in CRITICAL_TOPICS if counts.get(topic, 0) <= 0]
    warnings = [f'{topic} has no recorded messages'
                for topic in CORE_V4_TOPICS if counts.get(topic, 0) <= 0]
    events = read_events(os.path.join(session_dir, 'events.jsonl'))
    if not events:
        warnings.append('no track event markers were recorded')
    temperature = max_temperature(os.path.join(session_dir, 'system_health.csv'))
    if temperature is None:
        warnings.append('no valid temperature samples were recorded')
    return {
        'ok': not errors,
        'errors': errors,
        'warnings': warnings,
        'topic_counts': counts,
        'events': events,
        'max_temperature_c': temperature,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--json', action='store_true', dest='as_json')
    args = parser.parse_args(argv)
    report = verify(os.path.abspath(os.path.expanduser(args.session_dir)))
    if args.as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print('PASS' if report['ok'] else 'FAIL')
        for error in report['errors']:
            print(f'ERROR: {error}')
        for warning in report['warnings']:
            print(f'WARNING: {warning}')
        print(f"events: {len(report['events'])}")
        print(f"max_temperature_c: {report['max_temperature_c']}")
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
