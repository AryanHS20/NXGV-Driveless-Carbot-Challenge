#!/usr/bin/env python3
"""Summarize the compact event stream produced by record_autonomy_test.py."""

import argparse
import json
import math
from pathlib import Path
import statistics


def mean(values):
    return statistics.fmean(values) if values else None


def rounded(value, digits=4):
    return None if value is None else round(value, digits)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trial', type=Path, help='trial directory or events.jsonl')
    args = parser.parse_args()
    event_path = args.trial / 'events.jsonl' if args.trial.is_dir() else args.trial
    events = [json.loads(line) for line in event_path.read_text().splitlines() if line.strip()]
    modes = [event for event in events if event['topic'] == '/auto_mode']
    starts = [event['t'] for event in modes if event['data'] is True]
    ends = [event['t'] for event in modes if event['data'] is False]
    if not starts:
        raise SystemExit('no AUTO interval')
    start = starts[0]
    end = next((value for value in ends if value > start), events[-1]['t'])
    active = [event for event in events if start <= event['t'] <= end]

    def topic(name):
        return [event for event in active if event['topic'] == name]

    commands = topic('/cmd_vel_auto')
    duties = [abs(float(event['data']['linear_x'])) * 255.0 for event in commands]
    steers = [float(event['data']['steer_right_normalized']) for event in commands]
    moving = [(duty, steer) for duty, steer in zip(duties, steers) if duty >= 0.5]
    zero_times = [event['t'] for event, duty in zip(commands, duties) if duty < 0.5]
    direction_flips = 0
    prior_sign = 0
    for steer in steers:
        sign = 1 if steer > 0.05 else -1 if steer < -0.05 else 0
        if sign and prior_sign and sign != prior_sign:
            direction_flips += 1
        if sign:
            prior_sign = sign

    trajectories = []
    for event in topic('/v4_experimental/trajectory/status'):
        data = event['data'] if isinstance(event['data'], dict) else {}
        selected = data.get('selected_diagnostic_only')
        if not isinstance(selected, dict):
            continue
        trajectories.append({
            't': event['t'],
            'source': selected.get('steering_source'),
            'support': selected.get('minimum_support'),
            'lateral_m': selected.get('lateral_error_m'),
            'heading_rad': selected.get('heading_error_rad'),
            'curvature_per_m': selected.get('curvature_per_m'),
            'steer_rad': selected.get('command_steer_rad_diagnostic_only'),
            'clearance_m': selected.get('boundary_clearance_m'),
            'warning': data.get('last_warning') or '',
            'error': data.get('last_error') or '',
        })

    odometry = topic('/odom')
    odom_distance = 0.0
    for before, after in zip(odometry, odometry[1:]):
        odom_distance += math.hypot(
            float(after['data']['x']) - float(before['data']['x']),
            float(after['data']['y']) - float(before['data']['y']),
        )
    yaw = [float(event['data']['yaw']) for event in topic('/imu/rpy')
           if isinstance(event['data'], dict) and event['data'].get('yaw') is not None]

    masks = {}
    for name in (
        '/v4_experimental/bev/primary/coverage',
        '/v4_experimental/road/primary/candidate',
        '/v4_experimental/road/primary/connected',
        '/v4_experimental/road/primary/fused',
    ):
        samples = topic(name)
        fractions = [float(item['data']['nonzero_fraction']) for item in samples]
        centroids = [float(item['data']['centroid_x_px']) for item in samples
                     if item['data'].get('centroid_x_px') is not None]
        masks[name.rsplit('/', 1)[-1]] = {
            'samples': len(samples), 'fraction_min': rounded(min(fractions) if fractions else None),
            'fraction_mean': rounded(mean(fractions)),
            'fraction_max': rounded(max(fractions) if fractions else None),
            'centroid_x_min': rounded(min(centroids) if centroids else None, 2),
            'centroid_x_max': rounded(max(centroids) if centroids else None, 2),
        }

    dashboard = [event['data'] for event in topic('/dashboard_state')
                 if isinstance(event['data'], dict)]
    safety = [event['data'] for event in topic('/cmd_safety_status')
              if isinstance(event['data'], dict)]
    report = {
        'auto_interval': {'start_t': start, 'end_t': end, 'duration_sec': end - start},
        'commands': {
            'samples': len(commands),
            'duty_min': rounded(min(duties) if duties else None, 2),
            'duty_mean': rounded(mean(duties), 2),
            'duty_max': rounded(max(duties) if duties else None, 2),
            'moving_duty_mean': rounded(mean([item[0] for item in moving]), 2),
            'zero_samples': len(zero_times),
            'first_zero_t': rounded(min(zero_times) if zero_times else None, 3),
            'last_zero_t': rounded(max(zero_times) if zero_times else None, 3),
            'steer_min': rounded(min(steers) if steers else None),
            'steer_mean': rounded(mean(steers)),
            'steer_max': rounded(max(steers) if steers else None),
            'direction_flips_over_0_05': direction_flips,
        },
        'trajectory': {
            'samples': len(trajectories),
            'source_counts': {source: sum(row['source'] == source for row in trajectories)
                              for source in sorted({row['source'] for row in trajectories})},
            'minimum_support': rounded(min((row['support'] for row in trajectories
                                            if row['support'] is not None), default=None)),
            'minimum_clearance_m': rounded(min((row['clearance_m'] for row in trajectories
                                                if row['clearance_m'] is not None), default=None)),
            'maximum_abs_lateral_error_m': rounded(max((abs(row['lateral_m']) for row in trajectories
                                                        if row['lateral_m'] is not None), default=None)),
            'errors': sorted({row['error'] for row in trajectories if row['error']}),
            'warnings': sorted({row['warning'] for row in trajectories if row['warning']}),
            'timeline': trajectories,
        },
        'motion': {
            'odometry_path_length_m': rounded(odom_distance),
            'imu_yaw_start_deg': rounded(yaw[0] if yaw else None, 2),
            'imu_yaw_end_deg': rounded(yaw[-1] if yaw else None, 2),
            'imu_yaw_change_deg': rounded(yaw[-1] - yaw[0] if len(yaw) >= 2 else None, 2),
        },
        'masks': masks,
        'dashboard_states': sorted({item.get('state') for item in dashboard if item.get('state')}),
        'dashboard_stop_reasons': sorted({item.get('stop_reason') for item in dashboard
                                          if item.get('stop_reason')}),
        'safety_reasons': sorted({item.get('reason') for item in safety if item.get('reason')}),
    }
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
