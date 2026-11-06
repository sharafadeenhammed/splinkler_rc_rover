"""Hardware-free tests for the mission planner and guidance maths.

    python test_auto_plan.py

helpers/auto_plan.py is pure Python, so this runs on the dev box as well as on
the Pi.  The module is loaded by file path for consistency with
test_fusion_logic.py (importing the helpers package would drag in gpiozero).
"""
import importlib.util
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))


def load_helper(name):
    if "helpers" not in sys.modules:
        pkg = types.ModuleType("helpers")
        pkg.__path__ = [os.path.join(HERE, "helpers")]
        sys.modules["helpers"] = pkg
    full = f"helpers.{name}"
    if full not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            full, os.path.join(HERE, "helpers", f"{name}.py")
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[full] = module
        spec.loader.exec_module(module)
    return sys.modules[full]


plan = load_helper("auto_plan")

failures = []


def check(name, condition, detail=""):
    if condition:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        failures.append(name)


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def point_in_polygon(x, y, polygon):
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def test_rectangle_plan():
    print("rectangle plan")
    polygon = plan.rectangle_polygon(10.0, 10.0)
    strips = plan.plan_strips(polygon, 0.50, 0.10)
    spacing = plan.strip_spacing(0.50, 0.10)

    check("spacing honours overlap", close(spacing, 0.45), spacing)
    check("starts on the near edge", close(strips[0]["y"], 0.0), strips[0]["y"])
    check("one strip per row (rectangle)",
          len(strips) == len(plan.plan_rows(polygon, 0.50, 0.10)))

    last_y = strips[-1]["y"]
    check("far edge is wetted", last_y + 0.25 >= 10.0 - 1e-6, last_y)

    lengths = [strip["length_m"] for strip in strips]
    check("strip length equals area width", all(close(length, 10.0) for length in lengths),
          lengths[:3])

    directions = [strip["forward"] for strip in strips]
    check("directions alternate",
          all(directions[i] != directions[i + 1] for i in range(len(directions) - 1)),
          directions)

    # Consecutive strips must share an x end, otherwise the connector is diagonal.
    shared = True
    for first, second in zip(strips, strips[1:]):
        if not close(first["x1"], second["x0"], 1e-6):
            shared = False
    check("consecutive strips share an end", shared)

    y_gaps = [strips[i + 1]["y"] - strips[i]["y"] for i in range(len(strips) - 1)]
    check("row spacing is uniform", all(close(gap, spacing) for gap in y_gaps), y_gaps[:3])

    # Coverage: every point of the area should be within half a swath of a row.
    covered = True
    y = 0.0
    while y <= 10.0:
        if min(abs(y - strip["y"]) for strip in strips) > 0.25 + 1e-6:
            covered = False
            break
        y += 0.1
    check("every row is within half a swath of a pass", covered)


def test_concave_polygon():
    print("concave polygon")
    polygon = [[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [3.0, 3.0], [3.0, 8.0], [0.0, 8.0]]
    strips = plan.plan_strips(polygon, 0.5, 0.1)
    check("plan produced strips", len(strips) > 0)

    # A strip may sit exactly on a polygon edge (the near and far edges are
    # driven on purpose), so probe just inside the area rather than on it.
    top = max(point[1] for point in polygon)
    inside = True
    for strip in strips:
        mid_x = (strip["x0"] + strip["x1"]) / 2.0
        probe_y = strip["y"] - 1e-4 if strip["y"] >= top - 1e-6 else strip["y"] + 1e-4
        if not point_in_polygon(mid_x, probe_y, polygon):
            inside = False
            print(f"       strip midpoint outside: ({mid_x:.2f}, {strip['y']:.2f})")
    check("all strips lie inside the polygon", inside)

    wide_rows = [strip for strip in strips if strip["length_m"] > 5.0]
    narrow_rows = [strip for strip in strips if strip["length_m"] < 4.0]
    check("wide part of the L is covered", len(wide_rows) >= 4, len(wide_rows))
    check("narrow part of the L is covered", len(narrow_rows) >= 8, len(narrow_rows))

    check("every strip has real length", all(strip["length_m"] >= plan.MIN_SPAN_M for strip in strips))


def _rotate(points, angle, dx=0.0, dy=0.0):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return [[x * cos_a - y * sin_a + dx, x * sin_a + y * cos_a + dy] for x, y in points]


def test_mapped_polygon():
    """A work area drawn on the map must be planned where it was drawn."""
    print("mapped polygon")
    rectangle = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [0.0, 2.0]]
    angle = math.radians(30.0)
    polygon = _rotate(rectangle, angle, 5.0, -3.0)

    check("sweep follows the longest edge",
          close(abs(plan.longest_edge_angle(polygon)), angle, 1e-9),
          math.degrees(plan.longest_edge_angle(polygon)))

    strips = plan.plan_polygon(polygon, 0.50, 0.10)
    check("strips planned", len(strips) == 5, len(strips))
    check("passes run along the long edge",
          all(close(abs(plan.segment_heading(seg) - angle) % math.pi,
                    0.0, 1e-6) or close(abs(plan.segment_heading(seg) - angle) % math.pi,
                                         math.pi, 1e-6) for seg in strips),
          [round(math.degrees(plan.segment_heading(s)), 1) for s in strips])
    check("strip length is the area length",
          all(close(seg["length_m"], 4.0, 1e-6) for seg in strips),
          [round(seg["length_m"], 2) for seg in strips])

    inside_all = True
    for seg in strips:
        mid_x = (seg["x0"] + seg["x1"]) / 2.0
        mid_y = (seg["y0"] + seg["y1"]) / 2.0
        # probe just inside the area, the near/far edge lines sit on it
        if not (point_in_polygon(mid_x, mid_y, polygon)
                or point_in_polygon(mid_x, mid_y - 1e-3, polygon)
                or point_in_polygon(mid_x, mid_y + 1e-3, polygon)):
            inside_all = False
            print(f"       strip midpoint off the area: ({mid_x:.2f}, {mid_y:.2f})")
    check("strips stay on the drawn area", inside_all)

    # The plan keeps the map placement instead of starting at the origin.
    xs = [value for seg in strips for value in (seg["x0"], seg["x1"])]
    ys = [value for seg in strips for value in (seg["y0"], seg["y1"])]
    check("plan keeps the map position",
          min(xs) > 2.0 and min(ys) < -1.0,
          (round(min(xs), 2), round(min(ys), 2)))

    check("total length adds up",
          close(plan.total_planned_length(strips), 20.0, 1e-6),
          plan.total_planned_length(strips))

    # Reversed plan starts at the other corner and still links up.
    reversed_plan = plan.reverse_plan(strips)
    check("reverse keeps strip count", len(reversed_plan) == len(strips))
    check("reverse starts at the old end",
          close(reversed_plan[0]["x0"], strips[-1]["x1"], 1e-9) and
          close(reversed_plan[0]["y0"], strips[-1]["y1"], 1e-9))
    links = [plan.plan_connector(a, b) for a, b in zip(reversed_plan, reversed_plan[1:])]
    check("reversed strips still connect",
          all(link is not None and link["length_m"] < 0.7 for link in links),
          [round(link["length_m"], 2) if link else None for link in links])

    # Winding order must not matter.
    clockwise = list(reversed(polygon))
    cw_strips = plan.plan_polygon(clockwise, 0.50, 0.10)
    check("clockwise polygon plans the same distance",
          close(plan.total_planned_length(cw_strips), 20.0, 1e-6),
          plan.total_planned_length(cw_strips))


def test_normalize_and_world_transform():
    print("frames")
    polygon = [[5.0, 7.0], [9.0, 7.0], [9.0, 9.0], [5.0, 9.0]]
    normalized = plan.normalize_polygon(polygon)
    check("polygon min corner moved to origin",
          close(min(p[0] for p in normalized), 0.0) and close(min(p[1] for p in normalized), 0.0),
          normalized)

    segment = {"index": 0, "x0": 0.0, "y0": 0.0, "x1": 2.0, "y1": 0.0, "length_m": 2.0}
    world = plan.to_world(segment, 1.0, 2.0, math.pi / 2.0)
    check("x maps to +y when heading is +90 deg",
          close(world["x1"], 1.0, 1e-9) and close(world["y1"], 4.0, 1e-9),
          (world["x1"], world["y1"]))
    check("transformed length preserved", close(world["length_m"], 2.0))
    check("heading of transformed segment",
          close(plan.segment_heading(world), math.pi / 2.0))

    world0 = plan.to_world(segment, 3.0, 4.0, 0.0)
    check("identity rotation is a translation",
          close(world0["x0"], 3.0) and close(world0["y0"], 4.0) and
          close(world0["x1"], 5.0) and close(world0["y1"], 4.0))


def _distance_to_segment(px, py, segment):
    dx = segment["x1"] - segment["x0"]
    dy = segment["y1"] - segment["y0"]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - segment["x0"], py - segment["y0"])
    t = ((px - segment["x0"]) * dx + (py - segment["y0"]) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (segment["x0"] + t * dx), py - (segment["y0"] + t * dy))


def test_offset_area():
    """The area can be sized from the rover: forward/back/left/right metres."""
    print("offset area")
    polygon = plan.offsets_polygon(3.0, 1.0, 2.0, 0.5)
    check("rear-right corner", polygon[0] == [-1.0, -0.5], polygon[0])
    check("front-right corner", polygon[1] == [3.0, -0.5], polygon[1])
    check("front-left corner", polygon[2] == [3.0, 2.0], polygon[2])
    check("rear-left corner", polygon[3] == [-1.0, 2.0], polygon[3])
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    check("asymmetric box spans forward+back by left+right",
          close((max(xs) - min(xs)) * (max(ys) - min(ys)), 10.0), polygon)

    # The rover stands inside its own box when the offsets surround it.
    check("rover is inside the offset box", point_in_polygon(0.0, 0.0, polygon))

    # Heading east: forward is +x, left is +y.
    east = plan.offsets_world_polygon(3.0, 1.0, 2.0, 0.5, 4.0, 6.0, 0.0)
    check("east-facing box placed at the rover",
          close(east[0][0], 3.0) and close(east[0][1], 5.5) and
          close(east[2][0], 7.0) and close(east[2][1], 8.0), east)

    # Heading north: forward is +y, and left is -x (west).
    north = plan.offsets_world_polygon(3.0, 1.0, 2.0, 0.5, 4.0, 6.0, math.pi / 2.0)
    check("north-facing box turns with the rover",
          close(north[0][0], 4.5) and close(north[0][1], 5.0) and
          close(north[2][0], 2.0) and close(north[2][1], 9.0), north)

    # The plan covers exactly that box: 4 m of pass across 2.5 m of ground.
    strips = plan.plan_strips(polygon, 0.5, 0.1)
    check("offset box plans one pass per row", len(strips) == 6, len(strips))
    check("offset box pass length", all(close(strip["length_m"], 4.0) for strip in strips),
          [strip["length_m"] for strip in strips])
    check("offset box total distance", close(plan.total_planned_length(
        [plan.strip_segment(strip, i) for i, strip in enumerate(strips)]), 24.0))

    # Every point of the requested box must be within half a swath of a pass.
    segments = [plan.strip_segment(strip, i) for i, strip in enumerate(strips)]
    worst = 0.0
    y = 0.0
    while y <= 2.5:
        x = 0.0
        while x <= 4.0:
            worst = max(worst, min(_distance_to_segment(x, y, s) for s in segments))
            x += 0.25
        y += 0.25
    check("the whole offset box is wetted", worst <= 0.25 + 1e-6, worst)

    for bad in ((-1.0, 1.0, 2.0, 2.0), (0.2, 0.2, 2.0, 2.0), (3.0, 1.0, 0.1, 0.1)):
        try:
            plan.offsets_polygon(*bad)
            check(f"offsets {bad} rejected", False)
        except ValueError:
            check(f"offsets {bad} rejected", True)


def test_track_errors():
    print("track errors")
    segment = {"index": 0, "x0": 0.0, "y0": 0.0, "x1": 10.0, "y1": 0.0, "length_m": 10.0}
    along, cross = plan.track_errors(3.0, 0.0, segment)
    check("point on the line: along only", close(along, 3.0) and close(cross, 0.0), (along, cross))

    along, cross = plan.track_errors(3.0, 0.4, segment)
    check("point to the left is positive cross-track", close(cross, 0.4), cross)
    along, cross = plan.track_errors(3.0, -0.4, segment)
    check("point to the right is negative cross-track", close(cross, -0.4), cross)

    behind, _ = plan.track_errors(-2.0, 0.0, segment)
    check("behind the start is negative along-track", close(behind, -2.0), behind)

    # Same geometry rotated 90 degrees.
    rotated = {"index": 0, "x0": 0.0, "y0": 0.0, "x1": 0.0, "y1": 5.0, "length_m": 5.0}
    _, cross = plan.track_errors(-0.3, 2.0, rotated)
    check("left of a northbound line is negative x", close(cross, 0.3), cross)


def test_steering():
    print("steering")
    segment = {"index": 0, "x0": 0.0, "y0": 0.0, "x1": 10.0, "y1": 0.0, "length_m": 10.0}

    # Left of the line must steer right, and steering right on a differential
    # drive means the left wheel gets MORE duty than the right one.
    left, right, along, cross, err = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": 0.2, "heading_deg": 0.0}, 0.7)
    check("left of line turns right", left > right, (left, right))
    check("cross-track reported", close(cross, 0.2), cross)

    left, right, *_ = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": -0.2, "heading_deg": 0.0}, 0.7)
    check("right of line turns left", left < right, (left, right))

    # Heading too far clockwise (negative angle) must turn counter-clockwise.
    left, right, _, _, err = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": 0.0, "heading_deg": -10.0}, 0.7)
    check("heading error sign", err > 0, err)
    check("clockwise heading error steers left", left < right, (left, right))

    left, right, *_ = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": 0.0, "heading_deg": 10.0}, 0.7)
    check("counter-clockwise heading error steers right", left > right, (left, right))

    left, right, *_ = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": 20.0, "heading_deg": 0.0}, 0.7)
    check("duties stay within [min_duty, 1]",
          plan.MIN_DUTY - 1e-9 <= left <= 1.0 and plan.MIN_DUTY - 1e-9 <= right <= 1.0,
          (left, right))

    left, right, *_ = plan.steer_duties(
        segment, {"x_m": 2.0, "y_m": 0.0, "heading_deg": 0.0}, 0.7)
    check("on the line both wheels equal", close(left, 0.7) and close(right, 0.7), (left, right))


def test_connectors_and_trims():
    print("connectors and trims")
    first = {"index": 0, "x0": 0.0, "y0": 0.0, "x1": 10.0, "y1": 0.0, "length_m": 10.0}
    second = {"index": 1, "x0": 10.0, "y0": 0.45, "x1": 0.0, "y1": 0.45, "length_m": 10.0}
    connector = plan.plan_connector(first, second)
    check("connector exists", connector is not None)
    check("connector length is the row spacing", close(connector["length_m"], 0.45), connector["length_m"])
    check("connector heads across the rows",
          close(abs(plan.segment_heading(connector)), math.pi / 2.0), plan.segment_heading(connector))
    check("connector starts where strip one ends",
          close(connector["x0"], 10.0) and close(connector["y0"], 0.0))
    check("connector ends where strip two starts",
          close(connector["x1"], 10.0) and close(connector["y1"], 0.45))

    check("no connector when strips touch",
          plan.plan_connector(first, dict(first, x0=10.0, x1=10.0, length_m=0.0)) is None)

    trimmed = plan.trim_segment(first, 2.0, 1.0)
    check("trim keeps the segment direction",
          close(trimmed["x0"], 2.0) and close(trimmed["x1"], 3.0), (trimmed["x0"], trimmed["x1"]))
    check("trim length", close(trimmed["length_m"], 1.0), trimmed["length_m"])

    over = plan.trim_segment(first, 9.5, 5.0)
    check("trim clamps to the segment end", close(over["length_m"], 0.5), over["length_m"])

    past_end = plan.trim_segment(first, 12.0, 1.0)
    check("trim past the end is empty", close(past_end["length_m"], 0.0), past_end["length_m"])


def test_wrap_angle():
    print("angle wraparound")
    check("wrap pi maps to -pi", close(plan.wrap_angle(math.pi), -math.pi))
    check("wrap 3pi", close(abs(plan.wrap_angle(3.0 * math.pi)), math.pi))
    check("wrap -pi/2", close(plan.wrap_angle(-math.pi / 2.0), -math.pi / 2.0))
    check("wrap 2pi", close(plan.wrap_angle(2.0 * math.pi), 0.0, 1e-12))


def test_degenerate_input():
    print("degenerate input")
    try:
        plan.normalize_polygon([[0.0, 0.0], [1.0, 1.0]])
        check("two-point polygon rejected", False)
    except ValueError:
        check("two-point polygon rejected", True)

    sliver = [[0.0, 0.0], [0.05, 0.0], [0.05, 4.0], [0.0, 4.0]]
    check("sliver polygon plans nothing", plan.plan_strips(sliver, 0.5, 0.1) == [])


def main():
    test_rectangle_plan()
    test_concave_polygon()
    test_mapped_polygon()
    test_offset_area()
    test_normalize_and_world_transform()
    test_track_errors()
    test_steering()
    test_connectors_and_trims()
    test_wrap_angle()
    test_degenerate_input()

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("OK: auto_plan all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
