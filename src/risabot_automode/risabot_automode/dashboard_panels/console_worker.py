"""Fixed, headless entry point for expensive ZIP map/mission computations."""
import json
import os
from pathlib import Path
import sys


def main():
    # Direct script execution keeps the worker usable with source or colcon installs.
    import cv2
    import numpy as np
    import map_builder as mb
    import mission_planner as mp
    cv2.setNumThreads(1)
    if hasattr(os, 'nice'):
        os.nice(10)
    operation, root_text = sys.argv[1:]
    root = Path(root_text)
    data = json.loads((root / 'input.json').read_text(encoding='utf-8'))
    if operation == 'fit':
        transform, report = mb.fit_rigid(np.asarray(data['lap'], dtype=float),
                                        data['template'], data.get('initial'))
        result = dict(transform=list(map(float, transform)), report=report)
    elif operation == 'mission':
        road = mp.Road(data['template'])
        graph = mp.LaneGraph(road)
        poses = data['poses']
        routes = []
        for i in range(3):
            print(f'Planning leg {i + 1}/3', flush=True)
            if not mp.fit_check(road, poses[i])[0] or not mp.fit_check(road, poses[i + 1])[0]:
                routes.append(dict(ok=False, reason='Endpoint vehicle footprint is off the road'))
            else:
                routes.append(mp.plan_leg(road, graph, poses[i], poses[i + 1]))
        result = dict(routes=routes)
    else:
        raise ValueError('Unknown worker operation')
    def scalar(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(type(value).__name__)
    (root / 'result.json').write_text(json.dumps(result, default=scalar, allow_nan=False), encoding='utf-8')


if __name__ == '__main__':
    main()
