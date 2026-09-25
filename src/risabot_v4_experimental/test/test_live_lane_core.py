import math
from dataclasses import replace

import numpy as np
import pytest

from test_trajectory_core import profile, corridor_mask
from risabot_v4_experimental.trajectory_core import (
    PathPoint, TrajectoryConfig, TrajectoryError, VehicleGeometry,
    evaluate_steering_command,
)
from risabot_v4_experimental.live_lane_core import select_live_lane_arc


def run(left=0.0, mask=None, obstacles=(), initial=0.0):
    camera = profile()
    config = replace(TrajectoryConfig(), expected_lane_width_m=0.295)
    points = [PathPoint(float(x), left * (float(x) / 0.5)**2)
              for x in np.linspace(0.10, 0.85, 30)]
    return select_live_lane_arc(
        points, corridor_mask(camera, 0.40) if mask is None else mask,
        camera, VehicleGeometry(), config, obstacles, initial_steer=initial,
    )


def test_live_straight_and_mirrored_turns():
    straight, _ = run()
    left, _ = run(0.08)
    right, _ = run(-0.08)
    assert straight['command_steer_rad_diagnostic_only'] == 0.0
    assert left['command_steer_rad_diagnostic_only'] > 0
    assert right['command_steer_rad_diagnostic_only'] < 0
    assert left['command_steer_rad_diagnostic_only'] == pytest.approx(
        -right['command_steer_rad_diagnostic_only'])


def test_no_road_or_narrow_road_cannot_authorize_motion():
    for mask in (np.zeros_like(corridor_mask(profile())), corridor_mask(profile(), 0.06)):
        assert run(mask=mask)[0] is None


def test_obstacle_across_road_rejects_every_arc():
    obstacles = [(0.25, float(y)) for y in np.linspace(-0.5, 0.5, 60)]
    assert run(obstacles=obstacles)[0] is None


def test_selected_command_is_exactly_the_checked_command():
    selected, _ = run(0.08, initial=-0.1)
    assert selected is not None
    evaluated = evaluate_steering_command(
        selected['command_steer_rad_diagnostic_only'],
        corridor_mask(profile(), 0.40), profile(), VehicleGeometry(),
        replace(TrajectoryConfig(), expected_lane_width_m=0.295),
        initial_steer_rad=-0.1,
    )
    assert evaluated.road_blocked == evaluated.obstacle_blocked == 0
    assert selected['sent_command_endpoint_m']['left'] == evaluated.points[-1].y


def test_fresh_opposite_lane_replaces_previous_direction():
    selected, _ = run(-0.08, initial=0.15)
    assert selected['command_steer_rad_diagnostic_only'] < 0


@pytest.mark.parametrize('points', [
    [], [PathPoint(0.1, 0.0)] * 3,
    [PathPoint(0.1, 0), PathPoint(0.2, math.nan), PathPoint(0.3, 0)],
    [PathPoint(0.7, 0), PathPoint(0.8, 0), PathPoint(0.9, 0)],
])
def test_missing_invalid_or_unreachable_observations_stop(points):
    with pytest.raises(TrajectoryError):
        select_live_lane_arc(points, corridor_mask(profile()), profile(),
                             VehicleGeometry(), TrajectoryConfig())
