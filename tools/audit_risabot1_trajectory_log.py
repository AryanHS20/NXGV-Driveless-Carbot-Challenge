#!/usr/bin/env python3
"""Check recorded trajectory status for unsupported drive authorizations.

Usage: python3 tools/audit_risabot1_trajectory_log.py events.jsonl [--require-safe]
The recorder's compact status is enough for this check; this is not a camera
replay or proof that the mask matches the physical white line.
"""

import argparse
import json
from pathlib import Path


def audit(path):
    counts = {
        'trajectory_reports': 0,
        'valid_selections': 0,
        'unsafe_valid_selections': 0,
        'unknown_support_valid_selections': 0,
        'no_selection': 0,
    }
    first_unsafe = None
    with Path(path).open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            event = json.loads(line)
            if event.get('topic') != '/v4_experimental/trajectory/status':
                continue
            counts['trajectory_reports'] += 1
            selected = event.get('data', {}).get('selected_diagnostic_only')
            if not isinstance(selected, dict) or selected.get('valid') is not True:
                counts['no_selection'] += 1
                continue
            counts['valid_selections'] += 1
            blocked = selected.get('sent_command_road_blocked_steps')
            if blocked is None:
                counts['unknown_support_valid_selections'] += 1
                continue
            if blocked > 0 or selected.get('sent_command_obstacle_blocked_steps', 0) > 0:
                counts['unsafe_valid_selections'] += 1
                if first_unsafe is None:
                    first_unsafe = {
                        'event_time_sec': event.get('t'),
                        'line': line_number,
                        'road_blocked_steps': blocked,
                        'minimum_support': selected.get('sent_command_minimum_support'),
                        'obstacle_blocked_steps': selected.get('sent_command_obstacle_blocked_steps'),
                    }
    return {'file': str(path), 'counts': counts, 'first_unsafe': first_unsafe}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('--require-safe', action='store_true',
                        help='fail unless at least one valid supported selection exists and none are unsafe')
    args = parser.parse_args()
    result = audit(args.events)
    print(json.dumps(result, indent=2))
    counts = result['counts']
    if args.require_safe and (
        counts['valid_selections'] == 0
        or counts['unsafe_valid_selections'] > 0
        or counts['unknown_support_valid_selections'] > 0
    ):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
