"""Thread-safe draft map/mission workflow, reusing the supplied Setup Carbot tools.

Exports are isolated draft bundles. Nothing here writes ACTIVE, changes ROS
parameters, publishes commands, or claims a downstream consumer loaded a file.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import uuid

import numpy as np
import yaml

from . import map_builder as mb, mission_planner as mp
from .console_jobs import JobConflict, PlanningJobs


class PlanningSession:
    MAX_LAP = 6000

    def __init__(self, directory, jobs=None):
        self.directory = Path(directory)
        self.jobs = jobs or PlanningJobs()
        self.lock = threading.RLock()
        self.template = copy.deepcopy(mb.TEMPLATE)
        self.transform = [0.0, 0.0, 0.0]
        self.lap = []
        self.report = {}
        self.revision = 0
        self.map_bytes = None
        self.map_sha1 = None
        self.road = None
        self.poses = None
        self.routes = [None] * 3
        self.applied_job = None
        self.saved_bundle = None

    @staticmethod
    def numbers(values, count, limit=100):
        if not isinstance(values, (list, tuple)) or len(values) != count:
            raise ValueError(f'Expected {count} numeric coordinates')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or abs(v) > limit for v in values):
            raise ValueError('Coordinates must be finite and within the venue bounds')
        return list(map(float, values))

    def _collect(self):
        job = self.jobs.snapshot(include_result=True)
        if not job or job['id'] == self.applied_job or job['state'] in ('running', 'stopping'):
            return
        self.applied_job = job['id']
        if job['state'] != 'succeeded' or job['revision'] != self.revision:
            return  # Cancelled/old results may never resurrect a changed map or pose.
        if job['operation'] == 'fit':
            self.transform = job['result']['transform']
            self.report = job['result']['report']
        else:
            self.routes = job['result']['routes']
        self.revision += 1

    def _idle(self):
        self._collect()
        job = self.jobs.snapshot()
        if job and job['state'] in ('running', 'stopping'):
            raise JobConflict(f"{job['operation']} is running. Press Stop computation before editing or saving.")

    def _changed(self, map_changed=False):
        self.revision += 1
        self.routes = [None] * 3
        self.saved_bundle = None
        if map_changed:
            self.map_bytes = self.map_sha1 = self.road = self.poses = None
            self.report = {}

    def _report(self):
        if self.lap:
            lines = mb.centrelines(self.template, mb.FIT['sample_step_m'])
            self.report = mb.section_report(np.asarray(self.lap), self.transform,
                                            lines, mb.LineIndex(lines))

    def set_lap(self, points, frame):
        if frame != 'venue':
            raise ValueError('Lap must be in venue coordinates, without camera-to-map feedback')
        if not isinstance(points, list) or not 10 <= len(points) <= self.MAX_LAP:
            raise ValueError(f'Lap needs 10–{self.MAX_LAP} venue points; decimate longer recordings first')
        checked = [self.numbers(p, 2) for p in points]
        if np.linalg.norm(np.ptp(checked, axis=0)) < 0.2:
            raise ValueError('Lap has insufficient movement. Record the roads, not a stationary car.')
        with self.lock:
            self._idle()
            self.lap = checked
            self._changed(map_changed=True)
            self._report()

    def discard_lap(self):
        with self.lock:
            self._idle()
            self.lap = []
            self._changed(map_changed=True)

    def fit(self, refit=False):
        with self.lock:
            self._idle()
            if not self.lap:
                raise ValueError('Record or import a venue-frame lap first')
            initial = list(self.transform) if refit else None
            self._changed(map_changed=True)
            return self.jobs.start('fit', dict(lap=self.lap, template=self.template,
                                               initial=initial), self.revision)

    def move_handle(self, handle, point=None, reset=False):
        with self.lock:
            self._idle()
            choices = {hid: (p, kind) for hid, p, kind in mb.handles(self.template)}
            if handle not in choices:
                raise ValueError('Select a valid map handle first')
            default = {hid: p for hid, p, _ in mb.handles(mb.TEMPLATE)}[handle]
            target = list(default) if reset else self.numbers(point, 2, limit=12)
            # Bound raster/search allocation and reject accidentally dragging metres
            # off the competition drawing. Whole-map translation uses the transform.
            if math.dist(default, target) > 0.5:
                raise ValueError('A geometry handle may move at most 0.5 m from the template')
            template = copy.deepcopy(self.template)
            mb.move_handle(template, handle, choices[handle][1], np.asarray(target))
            arcs = [tuple(c.values()) for c in template['corners'].values()]
            arcs.append(tuple(template['roundabout'].values()))
            for points in arcs:
                circle = mb.circle_through(*points)
                if circle is None or not 0.02 <= circle[1] <= 5:
                    raise ValueError('Handle would make a degenerate or excessively large arc')
            self.template = template
            self._changed(map_changed=True)
            self._report()

    def save_map(self):
        with self.lock:
            self._idle()
            if (set(self.report) != set(mb.centrelines(self.template)) or
                    not all(r['ok'] for r in self.report.values())):
                raise ValueError('Every road section must be covered and pass the fit before saving')
            document = mb.to_yaml_dict(self.template, self.transform, self.report)
            self.map_bytes = yaml.safe_dump(document, sort_keys=False).encode('utf-8')
            self.map_sha1 = hashlib.sha1(self.map_bytes).hexdigest()
            # Reload the rounded export, exactly as the downstream planner does.
            bundle = self._new_bundle()
            path = bundle / 'track_map.yaml'
            path.write_bytes(self.map_bytes)
            template, fingerprint = mp.load_map(path)
            assert fingerprint == self.map_sha1
            self.road = mp.Road(template)
            self.poses = list(map(list, mp.default_poses(self.road)))
            self.routes = [None] * 3
            self.revision += 1
            self.saved_bundle = str(bundle)
            return dict(path=str(path), sha1=self.map_sha1, activated=False)

    def _new_bundle(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        bundle = self.directory / uuid.uuid4().hex
        bundle.mkdir()
        return bundle

    def edit_pose(self, index, action, point=None, degrees=None, snap=False):
        with self.lock:
            self._idle()
            if self.road is None:
                raise ValueError('Save a valid map before placing mission poses')
            if type(index) is not int or not 0 <= index <= 3:
                raise ValueError('Choose P0, P1, P2 or P3')
            pose = self.poses[index]
            if action == 'place':
                pose = self.numbers(point, 3, limit=100)
                pose[2] = mp.wrap(pose[2])
                if snap:
                    bay = self.road.bay_at(*pose[:2])
                    pose = self.road.bay_pose(bay, pose[2]) if bay else self.road.lane_snap(*pose)
            elif action == 'rotate':
                if degrees not in (-5, -1, 1, 5):
                    raise ValueError('Rotation must be -5, -1, 1 or 5 degrees')
                pose = [*pose[:2], mp.wrap(pose[2] + math.radians(degrees))]
            elif action == 'flip':
                pose = mp.flip(pose)
            elif action == 'reset':
                pose = mp.default_pose(self.road, index)
            else:
                raise ValueError('Unknown pose operation')
            self.poses[index] = list(pose)
            self._changed()

    def plan(self):
        with self.lock:
            self._idle()
            if self.road is None:
                raise ValueError('Save a valid map first')
            if not all(mp.fit_check(self.road, p)[0] for p in self.poses):
                raise ValueError('Every vehicle footprint must fit on the road before planning')
            return self.jobs.start('mission', dict(template=self.road.tpl, poses=self.poses), self.revision)

    def save_mission(self):
        with self.lock:
            self._idle()
            if self.map_bytes is None or not all(r and r.get('ok') for r in self.routes):
                raise ValueError('Save the map and successfully plan all three legs before saving the mission')
            doc = mp.mission_dict('track_map.yaml', self.map_sha1, self.poses, self.road, self.routes)
            if not all(p['fits'] for p in doc['poses']):
                raise ValueError('A mission pose is off the road')
            bundle = self._new_bundle()
            (bundle / 'track_map.yaml').write_bytes(self.map_bytes)
            (bundle / 'mission.yaml').write_text(yaml.safe_dump(doc, sort_keys=False), encoding='utf-8')
            # A manifest is written last. A partial bundle never looks complete.
            (bundle / 'draft.json').write_text(json.dumps(dict(
                complete=True, activated=False, map_sha1=self.map_sha1,
                missing=['runtime mission consumer', 'mission rules', 'vehicle geometry validation',
                         'calibration/race supervisor acknowledgement'])), encoding='utf-8')
            self.saved_bundle = str(bundle)
            return dict(path=str(bundle / 'mission.yaml'), map_sha1=self.map_sha1, activated=False)

    def snapshot(self):
        with self.lock:
            self._collect()
            fits = [dict(fits=bool(mp.fit_check(self.road, p)[0]),
                         margin_m=float(mp.fit_check(self.road, p)[1])) for p in self.poses] if self.poses else []
            lines = mb.centrelines(self.template, 0.04)
            return dict(revision=self.revision, transform=self.transform, lap=self.lap,
                        lines={key: value.tolist() for key, value in lines.items()},
                        handles=[dict(id=hid, point=list(map(float, p)), kind=kind)
                                 for hid, p, kind in mb.handles(self.template)],
                        report=self.report, map_sha1=self.map_sha1, poses=self.poses, fits=fits,
                        routes=self.routes, job=self.jobs.snapshot(), saved_bundle=self.saved_bundle,
                        activated=False, geometry=mb.CAR,
                        warning='Draft planning only. ZIP vehicle geometry is not hardware-validated; '
                                'the V4 runtime does not yet consume these map/mission exports.')

    def close(self):
        self.jobs.close()
