"""
Pure Pursuit path-tracking controller (pure Python, no ROS).

This module holds the tracking *algorithm* only. Given the car's current pose
in the map frame and a fixed array of waypoints (each carrying a target
velocity), it returns a ``(steering_angle, speed)`` command. Keeping it free of
ROS means it can be unit-tested with synthetic poses (see
``test/test_pure_pursuit.py``) and reused by any node.

The classic geometric pursuit pipeline:

1. find the waypoint closest to the car,
2. walk forward along the (closed-loop) path to the first waypoint that is at
   least one lookahead distance away -- this is the goal point,
3. rotate the goal point into the vehicle frame (x forward, y left),
4. solve the bicycle-model curvature that drives the rear axle through it,
   ``delta = atan2(2 * wheelbase * sin(alpha), lookahead)``,
5. take the speed straight from the goal waypoint's velocity profile.

The lookahead distance may be scaled with speed so the car looks further ahead
when moving fast; set ``lookahead_gain`` to 0 for a fixed lookahead.
"""

import numpy as np


class PurePursuitPlanner:
    """Stateless pure pursuit controller driven by a parameter namespace."""

    def __init__(self, params, waypoints):
        """
        Store tuning parameters and the path to track.

        :param params: an object (e.g. ``argparse.Namespace``) exposing the
            pure-pursuit tunables as attributes -- see ``config/params.yaml``.
        :param waypoints: an ``(N, 3)`` array of ``[x, y, velocity]`` rows in
            the map frame, ordered along the direction of travel.
        """
        self.params = params
        self.set_waypoints(waypoints)

        # The goal point chosen on the most recent plan() call, in the map
        # frame, exposed so a node can publish it as an RViz marker. None until
        # the first successful plan.
        self.last_goal_point = None
        self.last_nearest_index = 0

    def set_waypoints(self, waypoints):
        """Replace the tracked path with a new ``(N, 3)`` waypoint array."""
        self.waypoints = np.asarray(waypoints, dtype=float)
        if self.waypoints.ndim != 2 or self.waypoints.shape[1] < 3:
            raise ValueError(
                'waypoints must be an (N, 3) array of [x, y, velocity]')
        self.xy = self.waypoints[:, :2]
        self.velocities = self.waypoints[:, 2]

    def lookahead_for_speed(self, speed):
        """
        Return the lookahead distance to use at the given speed.

        Linearly scales between ``min_lookahead`` and ``max_lookahead`` around a
        base distance. With ``lookahead_gain == 0`` this collapses to the fixed
        ``lookahead_distance``.
        """
        p = self.params
        raw = p.lookahead_distance + p.lookahead_gain * speed
        return float(np.clip(raw, p.min_lookahead, p.max_lookahead))

    def find_nearest_index(self, x, y):
        """Return the index of the waypoint closest to ``(x, y)``."""
        deltas = self.xy - np.array([x, y])
        return int(np.argmin(np.einsum('ij,ij->i', deltas, deltas)))

    def find_goal_index(self, nearest_index, x, y, lookahead):
        """
        Return the index of the first waypoint at least ``lookahead`` away.

        Searches forward from ``nearest_index``, wrapping around the end of the
        array so a closed-loop track tracks continuously. Falls back to the
        farthest reachable point if none clears the lookahead (e.g. very sparse
        waypoints).
        """
        n = len(self.xy)
        for offset in range(n):
            i = (nearest_index + offset) % n
            dx = self.xy[i, 0] - x
            dy = self.xy[i, 1] - y
            if dx * dx + dy * dy >= lookahead * lookahead:
                return i
        # No point is beyond the lookahead radius: aim at the farthest one.
        return (nearest_index + n - 1) % n

    @staticmethod
    def to_vehicle_frame(goal_x, goal_y, x, y, yaw):
        """
        Rotate a map-frame point into the vehicle frame (x forward, y left).

        :returns: ``(local_x, local_y)`` of the point relative to the car.
        """
        dx = goal_x - x
        dy = goal_y - y
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        local_x = cos_y * dx + sin_y * dy
        local_y = -sin_y * dx + cos_y * dy
        return local_x, local_y

    def plan(self, x, y, yaw):
        """
        Turn one pose into a drive command.

        :param x: car x position in the map frame (metres).
        :param y: car y position in the map frame (metres).
        :param yaw: car heading in the map frame (radians).
        :returns: ``(steering_angle_rad, speed_mps)``, or ``None`` if there is
            no usable path (caller should then stop).
        """
        p = self.params
        if len(self.xy) == 0:
            return None

        nearest = self.find_nearest_index(x, y)
        self.last_nearest_index = nearest

        # Size the lookahead from the speed the profile wants here, so the
        # adaptive term reacts before we have actually sped up.
        target_speed = float(self.velocities[nearest])
        lookahead = self.lookahead_for_speed(target_speed)

        goal = self.find_goal_index(nearest, x, y, lookahead)
        goal_x, goal_y = self.xy[goal]
        self.last_goal_point = (goal_x, goal_y)

        local_x, local_y = self.to_vehicle_frame(goal_x, goal_y, x, y, yaw)

        # Safety: if the goal is behind the car (local_x < 0), advance one
        # more waypoint. This can happen at high speed or with sparse waypoints
        # when the nearest-point search returns a point the car has just passed.
        if local_x < 0.0:
            goal = (goal + 1) % len(self.xy)
            goal_x, goal_y = float(self.xy[goal, 0]), float(self.xy[goal, 1])
            self.last_goal_point = (goal_x, goal_y)
            local_x, local_y = self.to_vehicle_frame(goal_x, goal_y, x, y, yaw)

        # Distance to the goal point: the true lookahead used by the geometry.
        dist = float(np.hypot(local_x, local_y))
        if dist < 1e-6:
            return None

        # Bicycle-model pure pursuit. alpha is the bearing to the goal point;
        # the curvature that places the rear axle on a circle through it gives
        # delta = atan2(2 * L * sin(alpha), lookahead_distance).
        alpha = np.arctan2(local_y, local_x)
        steering = np.arctan2(
            2.0 * p.wheelbase * np.sin(alpha), dist)
        steering = float(np.clip(steering, -p.max_steer, p.max_steer))

        # Speed comes from the goal waypoint's velocity profile, globally
        # scaled and clamped to the car's limits.
        speed = float(self.velocities[goal]) * p.velocity_gain
        speed = float(np.clip(speed, p.min_speed, p.max_speed))

        return steering, speed
