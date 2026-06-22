"""
Waypoint CSV loading and saving (pure Python, no ROS).

A waypoint file is a delimited text file with one row per point. The loader is
deliberately forgiving: it auto-skips a non-numeric header row, accepts an
arbitrary delimiter (comma for the blaze_racer logger, semicolon for TUM-style
raceline exports), and lets the caller say which columns hold x, y and the
target velocity. Missing velocities are filled with a constant so an
x/y-only file still drives.

The canonical blaze_racer format produced by ``waypoint_logger_node`` is::

    x_m,y_m,velocity_mps

Functions return / accept plain NumPy arrays so they can be unit-tested
without a ROS node.
"""

import numpy as np


def _looks_numeric(token):
    """Return True if ``token`` parses as a float."""
    try:
        float(token)
        return True
    except ValueError:
        return False


def load_waypoints(path, x_col=0, y_col=1, v_col=2, delimiter=',',
                   default_velocity=1.0):
    """
    Load waypoints from a CSV/TSV file into an ``(N, 3)`` array.

    :param path: path to the waypoint file.
    :param x_col: column index of the x coordinate (metres).
    :param y_col: column index of the y coordinate (metres).
    :param v_col: column index of the target velocity (m/s). If the column is
        absent in a row, ``default_velocity`` is used instead.
    :param delimiter: field separator (``,`` or ``;`` are typical).
    :param default_velocity: velocity assigned when ``v_col`` is missing.
    :returns: an ``(N, 3)`` float array of ``[x, y, velocity]`` rows.
    :raises ValueError: if no valid rows are found.
    """
    rows = []
    with open(path, 'r') as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            fields = [f.strip() for f in line.split(delimiter)]
            # Skip a header line (any required field is non-numeric).
            if not _looks_numeric(fields[x_col]) \
                    or not _looks_numeric(fields[y_col]):
                continue
            x = float(fields[x_col])
            y = float(fields[y_col])
            if 0 <= v_col < len(fields) and _looks_numeric(fields[v_col]):
                v = float(fields[v_col])
            else:
                v = float(default_velocity)
            rows.append((x, y, v))

    if not rows:
        raise ValueError(f'No valid waypoint rows parsed from {path}')
    return np.array(rows, dtype=float)


def save_waypoints(path, waypoints, header='x_m,y_m,velocity_mps'):
    """
    Write an ``(N, 3)`` waypoint array to a CSV file.

    :param path: destination file path.
    :param waypoints: an ``(N, 3)`` array of ``[x, y, velocity]`` rows.
    :param header: header line written first (no trailing newline needed).
    """
    waypoints = np.asarray(waypoints, dtype=float)
    np.savetxt(path, waypoints, delimiter=',', header=header,
               comments='', fmt='%.6f')
