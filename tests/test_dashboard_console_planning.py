"""No ROS/hardware: fault, resource and revision boundaries for draft planning."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/risabot_automode'))
from risabot_automode.dashboard_panels import map_builder as mb
from risabot_automode.dashboard_panels import mission_planner as mp
from risabot_automode.dashboard_panels.console_jobs import JobConflict, PlanningJobs
from risabot_automode.dashboard_panels.console_planning import PlanningSession


class InertJobs:
    def __init__(self):
        self.job = None

    def snapshot(self, include_result=False):
        return copy.deepcopy(self.job)

    def start(self, operation, payload, revision):
        self.job = dict(id=str(revision), revision=revision, operation=operation,
                        state='running', result=None)
        return self.snapshot()

    def close(self):
        pass


@pytest.fixture
def session(tmp_path):
    return PlanningSession(tmp_path / 'drafts', InertJobs())


def complete_map(session):
    # Simulated measured coverage, deliberately confined to tests.
    session.report = {key: dict(ok=True, fitted=True, rms_m=0.001, covered=1.0)
                      for key in mb.centrelines(mb.TEMPLATE)}
    return session.save_map()


def test_no_default_pass_or_active_files(session):
    snap = session.snapshot()
    assert not snap['activated'] and not snap['report'] and snap['poses'] is None
    assert not session.directory.exists()
    with pytest.raises(ValueError, match='covered'):
        session.save_map()
    with pytest.raises(ValueError, match='Save a valid map'):
        session.edit_pose(0, 'reset')


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True, '1', 101])
def test_lap_rejects_invalid_points_without_mutating(session, bad):
    with pytest.raises(ValueError):
        session.set_lap([[bad, 0]] * 10, 'venue')
    assert session.revision == 0 and session.lap == []


def test_lap_limits_frame_and_motion(session):
    for points, frame in [([[0, 0]] * 6001, 'venue'), ([[0, 0]] * 10, 'track'),
                          ([[0, 0]] * 10, 'venue')]:
        with pytest.raises(ValueError):
            session.set_lap(points, frame)


def test_missing_coverage_blocks_save(session):
    session.report = {'loop': dict(ok=True), 'parking': dict(ok=False)}
    with pytest.raises(ValueError):
        session.save_map()


def test_saved_map_exact_hash_and_edit_invalidates_mission(session):
    result = complete_map(session)
    saved = Path(result['path']).read_bytes()
    assert result['sha1'] == hashlib.sha1(saved).hexdigest()
    assert not result['activated']
    hid, p, _ = mb.handles(session.template)[0]
    session.move_handle(hid, list(np.asarray(p) + [0.001, 0.001]))
    assert session.map_sha1 is None and session.poses is None
    assert Path(result['path']).read_bytes() == saved  # prior export stays recoverable
    with pytest.raises(ValueError):
        session.save_mission()


def test_oversized_geometry_edit_is_rejected(session):
    hid, p, _ = mb.handles(session.template)[0]
    with pytest.raises(ValueError, match='0.5 m'):
        session.move_handle(hid, list(np.asarray(p) + [2, 0]))
    assert session.revision == 0


def test_pose_flip_reuses_original_body_preserving_function(session):
    complete_map(session)
    old = session.poses[0]
    session.edit_pose(0, 'flip')
    assert np.allclose(mp.body_corners(old).mean(0), mp.body_corners(session.poses[0]).mean(0))
    assert all(route is None for route in session.routes)


def test_offroad_pose_prevents_planning(session):
    complete_map(session)
    session.edit_pose(0, 'place', [50, 50, 0])
    with pytest.raises(ValueError, match='footprint'):
        session.plan()


def test_busy_worker_blocks_edit_and_save_with_stop_guidance(session):
    session.jobs.start('fit', {}, session.revision)
    with pytest.raises(JobConflict, match='Stop computation'):
        session.discard_lap()
    with pytest.raises(JobConflict):
        session.save_map()


def test_old_worker_result_does_not_overwrite_new_revision(session):
    session.jobs.job = dict(id='old', operation='fit', state='succeeded', revision=-1,
                            result=dict(transform=[1, 2, 3], report={'fake': {}}))
    assert session.snapshot()['transform'] == [0, 0, 0]
    assert not session.report


def test_failed_worker_preserves_draft_and_can_be_retried(session):
    session.jobs.job = dict(id='bad', operation='fit', state='failed', revision=0, result=None)
    assert session.snapshot()['job']['state'] == 'failed'
    session.discard_lap()
    assert session.revision == 1


def test_mission_export_is_complete_bundle_never_activation(session):
    complete_map(session)
    session.poses = [list(mp.default_pose(session.road, 0))] * 4
    p = session.poses[0]
    session.routes = [dict(ok=True, length_m=0, roundabout_exits=[], sections=['start'],
                          pieces=[dict(kind='road', path=[[*p, 1], [*p, 1]])])] * 3
    result = session.save_mission()
    bundle = Path(result['path']).parent
    doc = yaml.safe_load(Path(result['path']).read_text())
    assert doc['map']['sha1'] == hashlib.sha1((bundle / 'track_map.yaml').read_bytes()).hexdigest()
    assert json.loads((bundle / 'draft.json').read_text())['activated'] is False
    assert not list(session.directory.rglob('ACTIVE'))


def wait_terminal(jobs):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        job = jobs.snapshot()
        if job['state'] not in ('running', 'stopping'):
            return job
        time.sleep(0.01)
    pytest.fail('Isolated test worker did not exit')


def test_actual_worker_crash_reports_failure_and_allows_restart():
    jobs = PlanningJobs(timeout=5)
    try:
        with pytest.raises(ValueError):
            jobs.start('arbitrary-script', {}, 0)
        jobs.start('fit', {}, 0)  # malformed worker input, not a sensor/robot action
        job = wait_terminal(jobs)
        assert job['state'] == 'failed' and job['error']
        jobs.start('fit', {}, 1)
        assert wait_terminal(jobs)['state'] == 'failed'
    finally:
        jobs.close()


def test_actual_worker_timeout_and_cancel_are_bounded():
    jobs = PlanningJobs(timeout=0.001)
    try:
        jobs.start('fit', {}, 0)
        assert 'timed out' in wait_terminal(jobs)['error']
        jobs.timeout = 5
        jobs.start('fit', {}, 1)
        with pytest.raises(JobConflict):
            jobs.start('mission', {}, 2)
        jobs.cancel()
        assert wait_terminal(jobs)['state'] == 'cancelled'
    finally:
        jobs.close()


def test_actual_worker_uses_zip_fit_and_reports_offroad_mission():
    jobs = PlanningJobs(timeout=20)
    try:
        lap = np.vstack(list(mb.centrelines(mb.TEMPLATE, 0.07).values())).tolist()
        jobs.start('fit', dict(template=mb.TEMPLATE, lap=lap, initial=[0, 0, 0]), 0)
        assert wait_terminal(jobs)['state'] == 'succeeded'
        result = jobs.snapshot(include_result=True)['result']
        assert np.allclose(result['transform'], [0, 0, 0], atol=0.01)
        assert set(result['report']) == set(mb.centrelines(mb.TEMPLATE))
        jobs.start('mission', dict(template=mb.TEMPLATE, poses=[[100, 100, 0]] * 4), 1)
        assert wait_terminal(jobs)['state'] == 'succeeded'
        assert all(not r['ok'] for r in jobs.snapshot(include_result=True)['result']['routes'])
    finally:
        jobs.close()
