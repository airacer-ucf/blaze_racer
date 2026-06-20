"""Unit tests for the ROS-free Follow-the-Gap planner."""

from argparse import Namespace

import numpy as np

from follow_the_gap.planner import FollowTheGapPlanner


# A synthetic 270 deg scan at 1 deg resolution: beam 0 at -135 deg.
ANGLE_MIN = np.radians(-135.0)
ANGLE_INC = np.radians(1.0)
N_BEAMS = 271


def make_params():
    """Return a parameter set sized for the synthetic scan above."""
    return Namespace(
        fov_degrees=180.0,
        bubble_radius=5,
        preprocess_conv_size=3,
        max_lidar_dist=10.0,
        safe_threshold=5,
        best_point_conv_size=20,
        max_steer=0.5,
        steering_gain=0.5,
        straights_steering_angle=0.1396,
        fast_steering_angle=0.0698,
        corners_speed=1.5,
        straights_speed=2.0,
        fast_speed=3.5,
    )


def test_centre_gap_steers_straight():
    """A deep corridor dead ahead yields near-zero steering and full speed."""
    # Near wall everywhere, with a deep open corridor straight ahead. Beam 135
    # of the full scan points straight forward (angle_min + 135 deg = 0).
    ranges = np.full(N_BEAMS, 1.0)
    ranges[125:146] = 10.0

    planner = FollowTheGapPlanner(make_params())
    steering, speed = planner.plan(ranges, ANGLE_MIN, ANGLE_INC)

    assert abs(steering) < np.radians(10.0)
    assert speed == make_params().fast_speed


def test_open_side_steers_toward_it():
    """A deep corridor on the left makes the planner steer left (positive)."""
    # Open corridor centred ~+45 deg (full-scan beam 180 = +45 deg).
    ranges = np.full(N_BEAMS, 1.0)
    ranges[170:191] = 10.0

    planner = FollowTheGapPlanner(make_params())
    steering, _ = planner.plan(ranges, ANGLE_MIN, ANGLE_INC)

    assert steering > 0.0


def test_empty_scan_returns_none():
    """An empty range array produces no command (caller should stop)."""
    planner = FollowTheGapPlanner(make_params())
    assert planner.plan(np.array([]), ANGLE_MIN, ANGLE_INC) is None
