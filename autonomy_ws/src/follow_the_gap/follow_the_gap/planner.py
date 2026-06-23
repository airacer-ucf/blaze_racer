"""
Follow-the-Gap reactive planner (pure Python, no ROS).

This module holds the gap-following *algorithm* only. It takes a raw range
array plus the scan geometry and returns a ``(steering_angle, speed)`` command.
Keeping it free of ROS means it can be unit-tested with synthetic scans (see
``test/test_planner.py``) and reused by any node.

The pipeline mirrors the classic RoboRacer "Follow the Gap" / Disparity Extender:

1. clamp ranges and crop to the forward field of view,
2. smooth to suppress single-beam noise,
3. zero out a safety "bubble" around the closest obstacle,
4. find the widest remaining gap and aim at the best point inside it,
5. pick a speed — either from the steering magnitude (slower in corners) or
   from the measured clearance straight ahead (farther → faster), depending
   on the ``use_distance_speed`` parameter.
"""

import numpy as np

from autonomy_core import lidar_utils


class FollowTheGapPlanner:
    """Stateless gap-following planner driven by a parameter namespace."""

    def __init__(self, params):
        """
        Store the tuning parameters.

        :param params: an object (e.g. ``argparse.Namespace``) exposing the
            follow-the-gap tunables as attributes — see ``config/params.yaml``.
        """
        self.params = params
        self.fov_rad = np.radians(params.fov_degrees)

    def plan(self, ranges, angle_min, angle_increment):
        """
        Turn one scan into a drive command.

        :param ranges: 1-D array of LiDAR ranges, beam 0 first.
        :param angle_min: bearing of ``ranges[0]`` in radians.
        :param angle_increment: angular step between beams, in radians.
        :returns: ``(steering_angle_rad, speed_mps)``, or ``None`` if no usable
            scan window is available (caller should then stop).
        """
        p = self.params

        # 1. Clamp impossible/inf distances, then keep only the forward FOV.
        ranges = lidar_utils.clip_ranges(ranges, p.max_lidar_dist)
        proc_ranges, angle_of_first = lidar_utils.crop_to_fov(
            ranges, angle_min, angle_increment, self.fov_rad)
        if proc_ranges.size == 0:
            return None

        # 2. Smooth, then re-clamp (the moving average can introduce overshoot).
        proc_ranges = lidar_utils.smooth(proc_ranges, p.preprocess_conv_size)
        proc_ranges = lidar_utils.clip_ranges(proc_ranges, p.max_lidar_dist)

        # 3. Zero a "bubble" around the closest point so we steer well clear of
        #    the nearest obstacle.
        closest = proc_ranges.argmin()
        min_index = max(closest - p.bubble_radius, 0)
        max_index = min(closest + p.bubble_radius, len(proc_ranges) - 1)

        # Measure front clearance *before* the bubble wipes out those beams.
        front_dist = self._front_clearance(proc_ranges, angle_of_first,
                                           angle_increment)

        proc_ranges[min_index:max_index] = 0

        # 4. Aim at the best point inside the widest free gap.
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        if gap_start is None or gap_end is None:
            non_zero = np.nonzero(proc_ranges)[0]
            if len(non_zero) > 0:
                best_idx = int(np.argmax(proc_ranges))
            else:
                # Everything is masked: aim straight ahead.
                best_idx = len(proc_ranges) // 2
        else:
            best_idx = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = lidar_utils.index_to_steering(
            best_idx, angle_of_first, angle_increment, p.max_steer,
            gain=p.steering_gain)

        # 5. Pick speed from either front clearance or steering magnitude.
        if getattr(p, 'use_distance_speed', False):
            speed = self._select_speed_by_distance(front_dist)
        else:
            speed = self._select_speed(steering_angle)
        return steering_angle, speed

    def _front_clearance(self, ranges, angle_of_first, angle_increment):
        """Return the mean range of beams within ``front_angle_width`` of 0 rad."""
        p = self.params
        half = getattr(p, 'front_angle_width', 0.1745) / 2.0  # default 10 deg
        n = len(ranges)
        # Map the ±half window around bearing 0 to array indices.
        i_center = int(round(-angle_of_first / angle_increment))
        i_half = max(1, int(round(half / angle_increment)))
        i_start = max(0, i_center - i_half)
        i_end = min(n, i_center + i_half + 1)
        window = ranges[i_start:i_end]
        if window.size == 0:
            return p.max_lidar_dist
        return float(np.mean(window))

    def _select_speed_by_distance(self, front_dist):
        """Linearly interpolate speed from front clearance (farther → faster)."""
        p = self.params
        d_min = getattr(p, 'dist_min_speed', 1.0)
        d_max = getattr(p, 'dist_max_speed', 5.0)
        v_min = getattr(p, 'distance_min_speed', p.corners_speed)
        v_max = getattr(p, 'distance_max_speed', p.fast_speed)
        t = np.clip((front_dist - d_min) / max(d_max - d_min, 1e-6), 0.0, 1.0)
        return float(v_min + t * (v_max - v_min))

    def _select_speed(self, steering_angle):
        """Pick a speed band from the steering magnitude (slower in corners)."""
        p = self.params
        if abs(steering_angle) > p.straights_steering_angle:
            return p.corners_speed
        if abs(steering_angle) > p.fast_steering_angle:
            return p.straights_speed
        return p.fast_speed

    def find_max_gap(self, free_space_ranges):
        """
        Return the (start, stop) indices of the widest non-zero gap.

        Prefers the longest run of free space that also clears the configured
        ``safe_threshold``; falls back to the longest run otherwise. Returns
        ``(None, None)`` when there is no free space at all.
        """
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)

        if slices is None or len(slices) == 0:
            return None, None

        max_len = 0
        chosen = None
        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len and sl_len > self.params.safe_threshold:
                max_len = sl_len
                chosen = sl

        if chosen is not None:
            return chosen.start, chosen.stop

        # No gap clears the threshold: use the largest one we have.
        largest = max(slices, key=lambda s: s.stop - s.start)
        return largest.start, largest.stop

    def find_best_point(self, start_i, end_i, ranges):
        """
        Return the index of the best aim point within ``[start_i, end_i)``.

        Uses a sliding-window average so the target favours the deepest part of
        the gap rather than a single noisy far reading. Small gaps just aim at
        the midpoint.
        """
        if end_i - start_i < self.params.best_point_conv_size:
            return start_i + (end_i - start_i) // 2

        window = np.ones(self.params.best_point_conv_size)
        averaged = np.convolve(ranges[start_i:end_i], window, 'same') \
            / self.params.best_point_conv_size
        return int(averaged.argmax()) + start_i
