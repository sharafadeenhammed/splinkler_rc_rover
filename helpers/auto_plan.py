"""Pure geometry and guidance maths for the autonomous lawnmower sweep.

Nothing here touches GPIO, threads or time, so it can be exercised on a
development machine (see test_auto_plan.py) and the controller only has to
drive.

Frame
─────
Angles are radians, counter-clockwise positive, wrapped to [-pi, pi).  That is
the convention the MPU6050 heading and PoseTracker already use: a positive
heading error means the rover is pointing left of where it should, so the left
wheel has to speed up to swing the nose clockwise back onto the row.

The sweep never needs to know where north is.  The controller captures the
heading at the moment Start is pressed as ``ref``; every row target is stored
relative to it:

    target_absolute = wrap(ref + radians(row_relative_deg))

Row 0 runs along the heading the rover had at Start (relative 0 deg), row 1
runs back along it (relative 180 deg), and the area extends to the side given
by ``turn_side``.
"""

import math

# Rows alternate between these two relative headings.
ROW_RELATIVE_FORWARD_DEG = 0.0
ROW_RELATIVE_RETURN_DEG = 180.0

# Wheel duty below which a JGB37-520 on this chassis buzzes without turning.
MIN_DRIVE_DUTY = 0.80


def wrap_rad(angle):
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def wrap_deg(angle):
    """Wrap an angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def circular_mean_rad(angles):
    """Mean of angles without the 359/1 deg seam, e.g. a stationary heading.

    Raises ValueError for an empty list or for angles that cancel out (a
    perfectly uniform spread has no meaningful mean).
    """
    values = [float(angle) for angle in angles]
    if not values:
        raise ValueError("circular mean needs at least one angle")
    sin_sum = sum(math.sin(angle) for angle in values)
    cos_sum = sum(math.cos(angle) for angle in values)
    if math.hypot(sin_sum, cos_sum) < 1e-9:
        raise ValueError("angles have no well-defined mean")
    return math.atan2(sin_sum, cos_sum)


def circular_mean_deg(angles):
    """Degrees version of :func:`circular_mean_rad`."""
    return math.degrees(circular_mean_rad([math.radians(value) for value in angles]))


def row_count(width_m, row_spacing_m):
    """Rows needed to work a ``width_m`` wide area at ``row_spacing_m``.

    This is the spec's ``width_m / row_spacing_m``, rounded up so a partial
    spacing still gets its own row; a width narrower than one spacing still
    needs one row.
    """
    width = float(width_m)
    spacing = float(row_spacing_m)
    if not (math.isfinite(width) and width > 0.0):
        raise ValueError("width must be a positive number of metres")
    if not (math.isfinite(spacing) and spacing > 0.0):
        raise ValueError("row spacing must be a positive number of metres")
    return max(1, int(math.ceil(width / spacing - 1e-9)))


def row_relative_deg(row_index):
    """Relative heading of a row: 0 deg on even rows, 180 deg on odd ones."""
    index = int(row_index)
    if index < 0:
        raise ValueError("row index must be zero or more")
    return ROW_RELATIVE_FORWARD_DEG if index % 2 == 0 else ROW_RELATIVE_RETURN_DEG


def turn_sign(turn_side):
    """Sign of a 90 deg turn in a CCW-positive frame: right = -1, left = +1."""
    side = str(turn_side or "").strip().lower()
    if side == "right":
        return -1.0
    if side == "left":
        return 1.0
    raise ValueError("turn_side must be 'right' or 'left'")


def target_rad(reference_rad, relative_deg):
    """Absolute heading for a relative target, wrapped to [-pi, pi)."""
    return wrap_rad(float(reference_rad) + math.radians(float(relative_deg)))


def heading_error_rad(current_rad, reference_rad, relative_deg):
    """Shortest signed error from the current heading to a relative target.

    Positive means the rover is pointing left (CCW) of the target.
    """
    return wrap_rad(float(current_rad) - target_rad(reference_rad, relative_deg))


def steer_duties(heading_error_rad_value, base_duty, gain_per_rad,
                 max_correction, min_duty=MIN_DRIVE_DUTY):
    """Differential wheel duties that pull the rover back onto a row.

    ``base_duty`` is the cruise duty of both wheels; the error adds to the
    left wheel and subtracts from the right, so a positive (nose-left) error
    speeds the left wheel and turns the rover clockwise.  The correction is
    capped by ``max_correction`` and by the room left above ``min_duty`` so
    neither wheel is pushed into the stall band.  Returns ``(left, right)``.
    """
    base = float(base_duty)
    floor = float(min_duty)
    if not math.isfinite(base):
        raise ValueError("base duty must be finite")
    if base < floor:
        base = floor
    limit = min(float(max_correction), max(0.0, base - floor))
    correction = float(gain_per_rad) * float(heading_error_rad_value)
    correction = max(-limit, min(limit, correction))
    left = max(-1.0, min(1.0, base + correction))
    right = max(-1.0, min(1.0, base - correction))
    return left, right


def row_progress(distance_m, row_length_m):
    """Fraction of a row covered, clamped to 0..1 (0 for a zero-length row)."""
    length = float(row_length_m)
    if length <= 0.0:
        return 1.0
    return max(0.0, min(1.0, float(distance_m) / length))


def sweep_plan(length_m, width_m, row_spacing_m, turn_side):
    """What the mission will do, for logging, telemetry and the dashboard.

    ``steps`` is the number of turn/step/turn manoeuvres, one fewer than the
    number of rows.  ``lateral_m`` is how far the rover moves sideways in
    total and ``total_path_m`` the planned driving distance, excluding turns.
    """
    length = float(length_m)
    width = float(width_m)
    spacing = float(row_spacing_m)
    rows = row_count(width, spacing)
    sign = turn_sign(turn_side)
    return {
        "rows": rows,
        "steps": rows - 1,
        "row_length_m": length,
        "lateral_m": (rows - 1) * spacing,
        "width_m": width,
        "row_spacing_m": spacing,
        "turn_sign": sign,
        "turn_side": "right" if sign < 0 else "left",
        "total_path_m": rows * length + (rows - 1) * spacing,
    }
