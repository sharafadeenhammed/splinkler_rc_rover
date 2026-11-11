"""Hardware-free tests for the auto-mode planning maths.

    python test_auto_plan.py         (or: PYTHONIOENCODING=utf-8 python ...)

helpers/auto_plan.py is pure Python, but importing the helpers package pulls in
gpiozero/pigpio, so the module is loaded by file path here (same trick as
test_fusion_logic.py).
"""

import importlib.util
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def load_plan():
    spec = importlib.util.spec_from_file_location(
        "auto_plan", os.path.join(HERE, "helpers", "auto_plan.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


plan = load_plan()


def test_wrap():
    assert plan.wrap_deg(190.0) == -170.0
    assert plan.wrap_deg(-180.0) == -180.0
    assert plan.wrap_deg(540.0) == -180.0
    assert abs(plan.wrap_rad(math.radians(370.0)) - math.radians(10.0)) < 1e-12
    print("PASS angle wrapping")


def test_circular_mean():
    assert abs(plan.circular_mean_deg([359.0, 1.0]) - 0.0) < 1e-6
    assert abs(plan.circular_mean_deg([10.0, 20.0, 30.0]) - 20.0) < 1e-6
    mean = plan.circular_mean_rad([math.radians(90.0)] * 5)
    assert abs(mean - math.pi / 2.0) < 1e-9
    try:
        plan.circular_mean_rad([])
    except ValueError:
        pass
    else:
        raise AssertionError("empty sample should be rejected")
    print("PASS circular mean (seam, empty)")


def test_rows():
    assert plan.row_count(3.0, 0.5) == 6
    assert plan.row_count(2.9, 0.5) == 6
    assert plan.row_count(0.0 + 0.2, 0.5) == 1
    assert plan.row_relative_deg(0) == 0.0
    assert plan.row_relative_deg(1) == 180.0
    assert plan.row_relative_deg(4) == 0.0
    print("PASS row count and alternating row headings")


def test_turn_convention():
    assert plan.turn_sign("right") == -1.0
    assert plan.turn_sign("left") == 1.0
    assert plan.turn_sign("RIGHT") == -1.0
    try:
        plan.turn_sign("sideways")
    except ValueError:
        pass
    else:
        raise AssertionError("bad turn_side should be rejected")
    print("PASS turn side signs")


def test_targets_and_errors():
    ref = math.radians(30.0)
    assert abs(plan.target_rad(ref, 0.0) - ref) < 1e-12
    assert abs(math.degrees(plan.target_rad(ref, 180.0)) - (-150.0)) < 1e-9

    # 10 deg left of the target is a positive error (CCW-positive frame).
    error = plan.heading_error_rad(math.radians(40.0), ref, 0.0)
    assert abs(math.degrees(error) - 10.0) < 1e-9
    # Return row: pointing along row 0 is 180 deg away from the row-1 target.
    error = plan.heading_error_rad(ref, ref, 180.0)
    assert abs(abs(math.degrees(error)) - 180.0) < 1e-9
    print("PASS relative targets and signed heading error")


def test_steering():
    left, right = plan.steer_duties(0.0, 0.85, 0.6, 0.12)
    assert abs(left - 0.85) < 1e-12 and abs(right - 0.85) < 1e-12

    left, right = plan.steer_duties(math.radians(10.0), 0.85, 0.6, 0.12)
    assert left > right, "nose-left error must speed the left wheel"
    assert abs((left + right) / 2.0 - 0.85) < 1e-12

    left, right = plan.steer_duties(math.radians(-10.0), 0.85, 0.6, 0.12)
    assert right > left

    # Huge error saturates at the room above the stall floor (0.85 - 0.80),
    # not at max_correction, so no wheel is pushed into the stall band.
    left, right = plan.steer_duties(math.pi, 0.85, 5.0, 0.5)
    assert abs(left - 0.90) < 1e-9, left
    assert abs(right - 0.80) < 1e-9, right
    assert right >= plan.MIN_DRIVE_DUTY - 1e-12

    # With headroom, the correction saturates at max_correction and at 1.0.
    left, right = plan.steer_duties(math.pi, 0.90, 5.0, 0.12)
    assert abs(left - 1.0) < 1e-9, left
    assert abs(right - 0.80) < 1e-9, right

    # A too-slow base duty is lifted to the stall floor.
    left, right = plan.steer_duties(0.0, 0.30, 0.6, 0.12)
    assert left == plan.MIN_DRIVE_DUTY == right
    print("PASS differential steering (sign, symmetry, limits, floor)")


def test_progress_and_plan():
    assert plan.row_progress(0.0, 5.0) == 0.0
    assert plan.row_progress(2.5, 5.0) == 0.5
    assert plan.row_progress(9.0, 5.0) == 1.0
    assert plan.row_progress(0.0, 0.0) == 1.0

    summary = plan.sweep_plan(4.0, 1.6, 0.4, "left")
    assert summary["rows"] == 4
    assert summary["steps"] == 3
    assert abs(summary["lateral_m"] - 1.2) < 1e-9
    assert abs(summary["total_path_m"] - (4 * 4.0 + 1.2)) < 1e-9
    assert summary["turn_sign"] == 1.0
    print("PASS row progress and sweep summary")


def test_state_machine_geometry():
    """The exact heading sequence a right-handed 3-row sweep should follow."""
    ref = math.radians(180.0)  # deliberately not north
    sign = plan.turn_sign("right")
    row0 = plan.row_relative_deg(0)
    row1 = plan.row_relative_deg(1)

    turn1 = plan.target_rad(ref, row0 + 90.0 * sign)
    turn2 = plan.target_rad(ref, row0 + 180.0 * sign)
    assert abs(math.degrees(plan.wrap_rad(turn1 - ref)) + 90.0) < 1e-9
    assert abs(math.degrees(plan.wrap_rad(turn2 - ref)) + 180.0) < 1e-9
    assert abs(math.degrees(plan.wrap_rad(plan.target_rad(ref, row1) - turn2))) < 1e-9

    # Left-handed sweep mirrors it.
    sign = plan.turn_sign("left")
    turn1 = plan.target_rad(ref, row0 + 90.0 * sign)
    assert abs(math.degrees(plan.wrap_rad(turn1 - ref)) - 90.0) < 1e-9
    print("PASS row/turn heading sequence (right and left, arbitrary reference)")


if __name__ == "__main__":
    test_wrap()
    test_circular_mean()
    test_rows()
    test_turn_convention()
    test_targets_and_errors()
    test_steering()
    test_progress_and_plan()
    test_state_machine_geometry()
    print("ALL AUTO PLAN TESTS PASSED")
