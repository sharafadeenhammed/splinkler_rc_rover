"""Mission planning and guidance maths for the autonomous coverage run.

Pure functions only: no GPIO, no threads, no sleeps.  Keeping the geometry out
of auto_controller.py means it can be exercised on a dev box
(test_auto_plan.py) and that the hardware module only has to drive.

Frames
──────
Area frame   metres, x runs along a strip, y across the strips.  The planner
             always normalises the polygon so its minimum corner is (0, 0);
             auto_controller rotates and translates the result into the
             rover's world frame when the mission starts, which is why the
             polygon's absolute placement on the map is not used.
World frame  metres, x east, y north — the frame PoseTracker reports.
Angles       radians, counter-clockwise from +x, wrapped to [-pi, pi).

Sign convention for guidance
────────────────────────────
`cross_track` is positive when the rover sits to the LEFT of the segment it is
following, so a positive cross-track error must produce a clockwise (right)
correction: the left wheel gets less duty than the right one.
"""

import math

# Width of ground one pass actually covers (the wetted band, not the chassis).
DEFAULT_SWATH_M = 0.50
# Fraction of each swath that is re-covered by the next pass, to hide GPS/pose
# error and spray spread.
DEFAULT_OVERLAP = 0.10
# Spans shorter than this are slivers from polygon edges, not worth a pass.
MIN_SPAN_M = 0.15
# Never command less than this duty: the gearmotors buzz without turning.
MIN_DUTY = 0.20


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def normalize_polygon(points):
    """Shift a polygon so its minimum corner sits at the origin."""
    if len(points) < 3:
        raise ValueError("A coverage polygon needs at least three points")
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    min_x, min_y = min(xs), min(ys)
    return [[x - min_x, y - min_y] for x, y in zip(xs, ys)]


def rectangle_polygon(width_m, height_m):
    """Build the polygon used when the area is a plain rectangle."""
    width_m = max(0.5, float(width_m))
    height_m = max(0.5, float(height_m))
    return [[0.0, 0.0], [width_m, 0.0], [width_m, height_m], [0.0, height_m]]


def strip_spacing(swath_m, overlap=DEFAULT_OVERLAP):
    """Distance between neighbouring strip centrelines."""
    swath = max(0.05, float(swath_m))
    overlap = max(0.0, min(0.9, float(overlap)))
    return swath * (1.0 - overlap)


def _line_spans(polygon, y, min_span_m=MIN_SPAN_M):
    """Sorted (left, right) x pairs where the horizontal line `y` is inside."""
    intersections = []
    count = len(polygon)
    for index in range(count):
        first = polygon[index]
        second = polygon[(index + 1) % count]
        y1, y2 = first[1], second[1]
        # Half-open rule: a vertex is counted once, so spans never double up.
        if (y1 <= y < y2) or (y2 <= y < y1):
            x = first[0] + (y - y1) * (second[0] - first[0]) / (y2 - y1)
            intersections.append(x)
    intersections.sort()
    spans = []
    for index in range(0, len(intersections) - 1, 2):
        left = intersections[index]
        right = intersections[index + 1]
        if right - left >= max(0.01, min_span_m):
            spans.append((left, right))
    return spans


def plan_rows(polygon, swath_m=DEFAULT_SWATH_M, overlap=DEFAULT_OVERLAP,
              min_span_m=MIN_SPAN_M):
    """Scan lines covering the polygon, as [(y, [(left, right), ...]), ...].

    The first line lies on the polygon edge the rover is assumed to start on,
    so no lead-in manoeuvre is needed; the last line is added explicitly when
    the regular spacing would leave the far edge unwetted.
    """
    polygon = normalize_polygon(polygon)
    spacing = strip_spacing(swath_m, overlap)
    swath = max(0.05, float(swath_m))
    height = max(point[1] for point in polygon)
    probe_limit = height - 1e-6

    rows = []
    y = 0.0
    while y <= height + 1e-9:
        spans = _line_spans(polygon, min(y, probe_limit), min_span_m)
        if spans:
            rows.append((y, spans))
        y += spacing

    if rows and rows[-1][0] < height - swath / 2.0:
        spans = _line_spans(polygon, probe_limit, min_span_m)
        if spans:
            rows.append((height, spans))
    return rows


def plan_strips(polygon, swath_m=DEFAULT_SWATH_M, overlap=DEFAULT_OVERLAP,
                min_span_m=MIN_SPAN_M):
    """Ordered boustrophedon strips in the area frame.

    Each strip is a dict with x0/y0/x1/y1/length_m plus `forward` (True when
    the pass runs towards +x).  Rows alternate direction, and rows split by a
    concave polygon are walked left-to-right then right-to-left so consecutive
    strips always share an end.
    """
    strips = []
    for row_index, (y, spans) in enumerate(plan_rows(polygon, swath_m, overlap, min_span_m)):
        forward = row_index % 2 == 0
        ordered = spans if forward else list(reversed(spans))
        for left, right in ordered:
            strips.append({
                "y": y,
                "x0": left if forward else right,
                "x1": right if forward else left,
                "length_m": right - left,
                "forward": forward,
            })
    return strips


def strip_segment(strip, index=0):
    """Convert a planned strip into a driveable segment in the area frame."""
    return {
        "index": index,
        "x0": float(strip["x0"]),
        "y0": float(strip["y"]),
        "x1": float(strip["x1"]),
        "y1": float(strip["y"]),
        "length_m": float(strip["length_m"]),
    }


def to_world(segment, origin_x, origin_y, heading_rad):
    """Rotate/translate an area-frame segment into the rover's world frame."""
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)

    def convert(x, y):
        return (
            origin_x + x * cos_h - y * sin_h,
            origin_y + x * sin_h + y * cos_h,
        )

    x0, y0 = convert(segment["x0"], segment["y0"])
    x1, y1 = convert(segment["x1"], segment["y1"])
    return {
        "index": segment.get("index", 0),
        "x0": x0,
        "y0": y0,
        "x1": x1,
        "y1": y1,
        "length_m": math.hypot(x1 - x0, y1 - y0),
    }


def segment_heading(segment):
    return math.atan2(segment["y1"] - segment["y0"], segment["x1"] - segment["x0"])


def segment_length(segment):
    return math.hypot(segment["x1"] - segment["x0"], segment["y1"] - segment["y0"])


def track_errors(px, py, segment):
    """Project a point onto a segment: (along_track_m, cross_track_m)."""
    dx = segment["x1"] - segment["x0"]
    dy = segment["y1"] - segment["y0"]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, 0.0
    ux, uy = dx / length, dy / length
    rx, ry = float(px) - segment["x0"], float(py) - segment["y0"]
    along = rx * ux + ry * uy
    cross = ux * ry - uy * rx        # positive = left of the direction of travel
    return along, cross


def steer_duties(segment, pose, base_speed, k_cross=1.10, k_heading=0.85,
                 min_duty=MIN_DUTY):
    """Wheel duties that pull the rover onto the segment.

    Returns (left, right, along_m, cross_m, heading_error_rad).  `base_speed`
    is a positive duty; reversing is handled by the caller.
    """
    along, cross = track_errors(pose["x_m"], pose["y_m"], segment)
    heading = math.radians(float(pose.get("heading_deg", 0.0)))
    heading_error = wrap_angle(segment_heading(segment) - heading)

    # Positive cross-track → left of the line → must turn clockwise (right).
    correction = k_heading * heading_error - k_cross * cross
    speed = max(0.0, float(base_speed))
    left = max(min_duty, min(1.0, speed - correction))
    right = max(min_duty, min(1.0, speed + correction))
    return left, right, along, cross, heading_error


def plan_connector(previous, following, min_length_m=0.02):
    """Short segment joining the end of one strip to the start of the next."""
    x0, y0 = previous["x1"], previous["y1"]
    x1, y1 = following["x0"], following["y0"]
    length = math.hypot(x1 - x0, y1 - y0)
    if length < min_length_m:
        return None
    return {
        "index": following.get("index", 0),
        "x0": x0, "y0": y0,
        "x1": x1, "y1": y1,
        "length_m": length,
    }


def trim_segment(segment, start_along_m, length_m):
    """Slice `length_m` off a segment, starting `start_along_m` along it."""
    length = segment_length(segment)
    if length <= 1e-9:
        return dict(segment)
    start = max(0.0, min(length, float(start_along_m)))
    length_m = max(0.0, float(length_m))
    end = min(length, start + length_m)
    ux = (segment["x1"] - segment["x0"]) / length
    uy = (segment["y1"] - segment["y0"]) / length
    x0 = segment["x0"] + ux * start
    y0 = segment["y0"] + uy * start
    x1 = segment["x0"] + ux * end
    y1 = segment["y0"] + uy * end
    return {
        "index": segment.get("index", 0),
        "x0": x0, "y0": y0,
        "x1": x1, "y1": y1,
        "length_m": max(0.0, end - start),
    }


def total_planned_length(strips):
    return sum(segment_length(strip) for strip in strips)


__all__ = [
    "DEFAULT_SWATH_M",
    "DEFAULT_OVERLAP",
    "MIN_SPAN_M",
    "MIN_DUTY",
    "wrap_angle",
    "normalize_polygon",
    "rectangle_polygon",
    "strip_spacing",
    "plan_rows",
    "plan_strips",
    "strip_segment",
    "to_world",
    "segment_heading",
    "segment_length",
    "track_errors",
    "steer_duties",
    "plan_connector",
    "trim_segment",
    "total_planned_length",
]
