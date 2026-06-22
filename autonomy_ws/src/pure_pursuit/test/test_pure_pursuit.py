"""Unit tests for the ROS-free Pure Pursuit planner and waypoint loader."""

from argparse import Namespace
import os
import tempfile

import numpy as np

from pure_pursuit.pure_pursuit import PurePursuitPlanner
from pure_pursuit.waypoint_loader import load_waypoints, save_waypoints


def make_params(**overrides):
    """Return a parameter set with sane defaults for the tests."""
    params = dict(
        wheelbase=0.33,
        lookahead_distance=1.0,
        lookahead_gain=0.0,
        min_lookahead=0.5,
        max_lookahead=3.0,
        max_steer=0.4,
        velocity_gain=1.0,
        min_speed=0.0,
        max_speed=7.0,
    )
    params.update(overrides)
    return Namespace(**params)


def straight_waypoints():
    """A straight line of waypoints along +x at 2.0 m/s."""
    xs = np.linspace(0.0, 10.0, 51)
    return np.column_stack([xs, np.zeros_like(xs), np.full_like(xs, 2.0)])


def test_straight_path_drives_straight():
    """On a straight line, steering is ~0 and speed matches the profile."""
    planner = PurePursuitPlanner(make_params(), straight_waypoints())
    steering, speed = planner.plan(0.0, 0.0, 0.0)
    assert abs(steering) < 1e-3
    assert abs(speed - 2.0) < 1e-6


def test_goal_to_the_left_steers_left():
    """A goal point offset to the left yields positive (left) steering."""
    planner = PurePursuitPlanner(make_params(), straight_waypoints())
    # Sit below the line and heading along +x: the path is to our left.
    steering, _ = planner.plan(0.0, -0.5, 0.0)
    assert steering > 0.0


def test_goal_to_the_right_steers_right():
    """A goal point offset to the right yields negative (right) steering."""
    planner = PurePursuitPlanner(make_params(), straight_waypoints())
    steering, _ = planner.plan(0.0, 0.5, 0.0)
    assert steering < 0.0


def test_steering_is_clamped():
    """Steering never exceeds the configured mechanical limit."""
    params = make_params(max_steer=0.2)
    planner = PurePursuitPlanner(params, straight_waypoints())
    steering, _ = planner.plan(0.0, -5.0, 0.0)
    assert abs(steering) <= 0.2 + 1e-9


def test_speed_is_clamped_and_scaled():
    """velocity_gain scales the profile and max_speed clamps it."""
    params = make_params(velocity_gain=10.0, max_speed=5.0)
    planner = PurePursuitPlanner(params, straight_waypoints())
    _, speed = planner.plan(0.0, 0.0, 0.0)
    assert abs(speed - 5.0) < 1e-6


def test_goal_respects_lookahead():
    """The chosen goal point is at least one lookahead away."""
    params = make_params(lookahead_distance=2.0)
    planner = PurePursuitPlanner(params, straight_waypoints())
    planner.plan(0.0, 0.0, 0.0)
    gx, gy = planner.last_goal_point
    assert np.hypot(gx, gy) >= 2.0 - 1e-6


def test_loader_round_trip():
    """save_waypoints then load_waypoints reproduces the data."""
    wps = straight_waypoints()
    tmp = os.path.join(tempfile.gettempdir(), 'pp_test_wp.csv')
    save_waypoints(tmp, wps)
    loaded = load_waypoints(tmp)
    assert loaded.shape == wps.shape
    assert np.allclose(loaded, wps, atol=1e-5)
    os.remove(tmp)


def test_loader_skips_header_and_fills_velocity():
    """A header row is skipped and a missing velocity column is filled."""
    tmp = os.path.join(tempfile.gettempdir(), 'pp_test_wp2.csv')
    with open(tmp, 'w') as handle:
        handle.write('x_m,y_m\n')      # header, only two columns
        handle.write('1.0,2.0\n')
        handle.write('3.0,4.0\n')
    loaded = load_waypoints(tmp, default_velocity=1.5)
    assert loaded.shape == (2, 3)
    assert np.allclose(loaded[:, 2], 1.5)
    os.remove(tmp)
