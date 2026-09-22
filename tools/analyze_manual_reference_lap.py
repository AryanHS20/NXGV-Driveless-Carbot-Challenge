#!/usr/bin/env python3
"""Summarize a manual reference lap recorded by record_manual_reference_lap.py."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import statistics


MOVE_THRESHOLD = 0.12


def mean(values):
    return statistics.fmean(values) if values else None


def rounded(value, digits=4):
    return None if value is None else round(value, digits)


def nearest(events, times, stamp):
    index = bisect_left(times, stamp)
    choices = [i for i in (index - 1, index) if 0 <= i < len(events)]
    return min((events[i] for i in choices), key=lambda item: abs(item['t'] - stamp))


def moving_intervals(commands):
    intervals = []
    start = last = None
    for event in commands:
        moving = abs(float(event['data']['linear_x'])) > MOVE_THRESHOLD
        if moving:
            if start is None:
                start = event['t']
            last = event['t']
        elif start is not None and event['t'] - last > 0.6:
            if last - start >= 0.5:
                intervals.append((start, last))
            start = last = None
    if start is not None and last - start >= 0.5:
        intervals.append((start, last))
    return intervals


def lap_windows(commands, count):
    intervals = moving_intervals(commands)
    if not intervals:
        return []
    gaps = sorted(
        ((right[0] - left[1], left[1], right[0])
         for left, right in zip(intervals, intervals[1:])),
        reverse=True,
    )
    separators = sorted((end, start) for _, end, start in gaps[:max(0, count - 1)])
    windows = []
    start = intervals[0][0]
    for end, following in separators:
        windows.append((start, end))
        start = following
    windows.append((start, intervals[-1][1]))
    return windows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', type=Path)
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.recording / 'events.jsonl' if args.recording.is_dir() else args.recording
    topics = {}
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                event = json.loads(line)
                topics.setdefault(event['topic'], []).append(event)

    commands = topics.get('/cmd_vel', [])
    trajectories = topics.get('/v4_experimental/trajectory/status', [])
    road = topics.get('/v4_experimental/road/status', [])
    imu = topics.get('/imu/rpy', [])
    if not commands or not trajectories or not road:
        raise SystemExit('manual command, trajectory, or road events are missing')
    command_times = [item['t'] for item in commands]
    road_times = [item['t'] for item in road]
    reports = []

    for index, (start, end) in enumerate(lap_windows(commands, args.laps), 1):
        lap_commands = [item for item in commands if start <= item['t'] <= end]
        throttle = [float(item['data']['linear_x']) for item in lap_commands]
        steering = [float(item['data']['steer_right_normalized']) for item in lap_commands]
        forward = reverse = stopped = 0.0
        for before, after in zip(lap_commands, lap_commands[1:]):
            duration = after['t'] - before['t']
            value = float(before['data']['linear_x'])
            if value > MOVE_THRESHOLD:
                forward += duration
            elif value < -MOVE_THRESHOLD:
                reverse += duration
            else:
                stopped += duration

        usable = high_support = observed_fallback = visual_loss = 0
        agreement = agreement_samples = 0
        for event in trajectories:
            if not start <= event['t'] <= end or not isinstance(event['data'], dict):
                continue
            selected = event['data'].get('selected_diagnostic_only')
            if not isinstance(selected, dict):
                continue
            usable += 1
            support = float(selected.get('minimum_support', 0.0))
            road_event = nearest(road, road_times, event['t'])
            corridor = road_event['data'].get('corridor', {}).get('primary', [])
            observed_fraction = (
                sum(bool(row.get('left_boundary_observed'))
                    or bool(row.get('right_boundary_observed')) for row in corridor)
                / len(corridor) if corridor else 0.0
            )
            if support >= 0.75:
                high_support += 1
            elif observed_fraction >= 0.50:
                observed_fallback += 1
            else:
                visual_loss += 1
            manual = float(nearest(commands, command_times, event['t'])['data'][
                'steer_right_normalized'
            ])
            try:
                predicted = (
                    0.90 * float(selected['feedforward_steer_rad'])
                    + 0.85 * float(selected['heading_error_rad'])
                    + math.atan2(
                        1.10 * float(selected['control_lateral_error_m']),
                        max(0.12, float(selected['evaluation_forward_m'])),
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue
            if abs(manual) > 0.15 and abs(predicted) > 0.10:
                agreement_samples += 1
                agreement += int(manual * predicted > 0.0)

        pitches = [float(item['data']['pitch']) for item in imu
                   if start <= item['t'] <= end and isinstance(item['data'], dict)]
        duration = end - start
        reports.append({
            'lap': index,
            'start_t': rounded(start, 3),
            'end_t': rounded(end, 3),
            'duration_sec': rounded(duration, 3),
            'manual_control': {
                'forward_percent': rounded(100.0 * forward / duration, 1),
                'reverse_percent': rounded(100.0 * reverse / duration, 1),
                'stopped_percent': rounded(100.0 * stopped / duration, 1),
                'mean_abs_throttle': rounded(mean([abs(value) for value in throttle]), 3),
                'mean_abs_steering': rounded(mean([abs(value) for value in steering]), 3),
                'full_lock_percent': rounded(
                    100.0 * sum(abs(value) > 0.90 for value in steering) / len(steering), 1
                ),
                'maximum_pitch_deg': rounded(max(pitches) if pitches else None, 2),
            },
            'perception_and_control': {
                'trajectory_samples': usable,
                'normal_centerline_percent': rounded(100.0 * high_support / usable, 1),
                'observed_boundary_fallback_percent': rounded(
                    100.0 * observed_fallback / usable, 1
                ),
                'true_visual_loss_percent': rounded(100.0 * visual_loss / usable, 1),
                'manual_direction_agreement_percent': rounded(
                    100.0 * agreement / agreement_samples if agreement_samples else None, 1
                ),
                'direction_agreement_samples': agreement_samples,
            },
        })

    result = {
        'recording': str(source.parent),
        'requested_laps': args.laps,
        'detected_laps': len(reports),
        'laps': reports,
    }
    rendered = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(rendered + '\n', encoding='utf-8')
    print(rendered)


if __name__ == '__main__':
    main()
