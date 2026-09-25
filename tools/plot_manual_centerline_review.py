#!/usr/bin/env python3
"""Show the simulator route and measured, vehicle-relative manual-lap centerlines.

The recording lacks usable global distance; do not draw its local observations
as a measured map route. This figure keeps planned and observed data separate.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def local_center(row, width_m=0.295):
    left = row.get('left_boundary_observed', False)
    right = row.get('right_boundary_observed', False)
    if left and right:
        return 0.5 * (float(row['left_boundary_m']) + float(row['right_boundary_m']))
    if left:
        return float(row['left_boundary_m']) - 0.5 * width_m
    if right:
        return float(row['right_boundary_m']) + 0.5 * width_m
    return None


def sample_lap(events_path, end_sec=198.7):
    events = [json.loads(line) for line in Path(events_path).open(encoding='utf-8')]
    statuses = [entry for entry in events
                if entry['topic'] == '/v4_experimental/road/status'
                and entry['t'] < end_sec]
    commands = [entry for entry in events
                if entry['topic'] == '/cmd_vel' and entry['t'] < end_sec]
    seconds = np.arange(0, int(end_sec) + 1)
    values = []
    support = []
    for second in seconds:
        status = min(statuses, key=lambda event: abs(event['t'] - second))
        command = min(commands, key=lambda event: abs(event['t'] - second))
        moving = (abs(float(command['data'].get('linear_x', 0.0))) > 0.02
                  and abs(command['t'] - second) < 0.4)
        rows = status['data'].get('corridor', {}).get('primary', [])
        observed = [(abs(float(row['forward_m']) - 0.60), row)
                    for row in rows
                    if 0.45 <= float(row['forward_m']) <= 0.80
                    and local_center(row) is not None]
        if not moving or not observed or abs(status['t'] - second) > 0.9:
            values.append(np.nan)
        else:
            values.append(local_center(min(observed, key=lambda pair: pair[0])[1]) * 100.0)
        support.append(sum(local_center(row) is not None for row in rows) / len(rows)
                       if rows else 0.0)
    return seconds, np.asarray(values), np.asarray(support)


def plot(simulator_json, events_jsonl, output):
    with Path(simulator_json).open(encoding='utf-8-sig') as stream:
        simulator = json.load(stream)
    times, lateral_cm, support = sample_lap(events_jsonl)
    fig = plt.figure(figsize=(14, 9), layout='constrained')
    grid = fig.add_gridspec(2, 1, height_ratios=[1.5, 1.0])
    ax = fig.add_subplot(grid[0])
    for lane in simulator['paths']:
        xy = np.array([(point['x'], point['y']) for point in lane])
        ax.plot(xy[:, 0], xy[:, 1], color='#cbd5df', linewidth=17,
                solid_capstyle='round', alpha=0.9, zorder=1)
    for index, route in enumerate(simulator['routes']):
        xy = np.array([(point['x'], point['y']) for point in route])
        color = '#007f78' if index == 0 else '#e69b22'
        ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=2.2,
                label=f'Simulator mission {index + 1} planned centerline', zorder=3)
        ax.annotate('', xy=xy[min(80, len(xy)-1)], xytext=xy[min(60, len(xy)-1)],
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=2))
    start = simulator['start']
    light = simulator['light']
    ax.scatter([start['x']], [start['y']], s=100, marker='*', color='#006a5d', zorder=5)
    ax.annotate('START', (start['x'], start['y']), xytext=(7.15, 0.75),
                arrowprops=dict(arrowstyle='->', color='#49515c'))
    ax.scatter([light['x']], [light['y']], s=65, marker='s', color='#af331d', zorder=5)
    ax.annotate('Traffic-light handoff', (light['x'], light['y']), xytext=(5.25, 4.20),
                arrowprops=dict(arrowstyle='->', color='#49515c'))
    for label, x, y in [('Lane change', 3.95, 1.10), ('Roundabout', 2.4, 0.8),
                        ('Tunnel', 0.45, 2.55), ('Hill', 3.60, 4.75),
                        ('Bumper', 5.95, 4.75)]:
        ax.text(x, y, label, ha='center', va='center', fontsize=9,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.88, pad=2), zorder=6)
    ax.set(xlim=(0, simulator['bounds']['w']), ylim=(0, simulator['bounds']['h']),
           xlabel='Simulator map x (m)', ylabel='Simulator map y (m)',
           title='Whole-course planned centerlines from Carbot_Simulator')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(color='#e6edf3', linewidth=.5)
    ax.legend(loc='lower right', fontsize=9)

    ax2 = fig.add_subplot(grid[1])
    ax2.axvspan(116, 124, color='#edc8ad', alpha=.55, label='Green bend in manual video')
    ax2.plot(times, lateral_cm, color='#2066a3', linewidth=1.5, marker='.', markersize=3,
             label='Recorded center target ~0.6 m ahead')
    ax2.axhline(0, color='#66717e', linewidth=.8)
    ax2.set(xlim=(45, 170), xlabel='Time in manual recording (s)',
            ylabel='Target left of car (cm)',
            title='Camera-derived local centerline while the manual car was moving')
    ax2.grid(color='#e6edf3', linewidth=.6)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.text(.01, -.30,
             'Gaps mean no throttle command or no usable observed white edge. This is vehicle-relative perception, '
             'not a globally aligned recording.\n'
             'Risabot 1 odometry reported only ~3.3 m for this full manual run, '
             'so it cannot locate these samples on the course map.',
             transform=ax2.transAxes, fontsize=9, color='#455463')
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, bbox_inches='tight')
    print(output)
    print(f'usable 1-second samples: {np.isfinite(lateral_cm).sum()} / {len(times)}')
    print(f'mean observed-edge fraction: {np.mean(support):.3f}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('simulator_json')
    parser.add_argument('events_jsonl')
    parser.add_argument('output')
    args = parser.parse_args()
    plot(args.simulator_json, args.events_jsonl, args.output)
