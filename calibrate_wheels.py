"""Interactive JGB37-520 wheel and encoder calibration.

Run on the Raspberry Pi with the rover lifted safely off the ground:

    python calibrate_wheels.py

The script tests each wheel independently, displays signed encoder counts,
and can measure counts for one complete output-wheel revolution.  The signed
count is useful for finding an encoder A/B polarity mismatch; the physical
direction must still be confirmed by watching the wheel.
"""

import argparse
import sys
import time

from helpers.encoder_controller import (
    LEFT_ENCODER,
    RIGHT_ENCODER,
    get_encoder_ticks,
    reset_encoders,
)
from helpers.motor_controls import drive_tank, stop


TEST_SPEED = 1
TEST_SECONDS = 1.5


def _require_hardware():
    if LEFT_ENCODER is None or RIGHT_ENCODER is None:
        raise RuntimeError(
            "Both encoders are unavailable. Start pigpiod and check the encoder wiring."
        )


def _drive_wheel(side, speed):
    """Drive only one wheel; positive speed means logical forward."""
    if side == "left":
        drive_tank(speed, 0)
    else:
        drive_tank(0, speed)


def _test_direction(side, speed, seconds):
    reset_encoders()
    _drive_wheel(side, speed)
    started = time.monotonic()
    try:
        while time.monotonic() - started < seconds:
            time.sleep(0.02)
    finally:
        stop()

    left_ticks, right_ticks = get_encoder_ticks()
    return left_ticks if side == "left" else right_ticks


def test_direction(side, speed, seconds):
    print(f"\n{side.title()} wheel: logical FORWARD for {seconds:.1f}s")
    input("Press ENTER, then watch the wheel...")
    ticks = _test_direction(side, abs(speed), seconds)
    observed = input("Did it move physically forward, reverse, or stop? [f/r/s]: ").strip().lower()
    observed = {"forward": "f", "reverse": "r", "backward": "r", "stop": "s"}.get(observed, observed)

    print(f"  Encoder count: {ticks:+d} ticks")
    if observed == "f" and ticks < 0:
        print("  RESULT: physical direction is correct; encoder sign is inverted.")
    elif observed == "r" and ticks > 0:
        print("  RESULT: motor polarity or logical direction is inverted.")
    elif observed == "s":
        print("  RESULT: check motor power, H-bridge wiring, and wheel clearance.")
    else:
        print("  RESULT: direction and encoder sign agree with the current configuration.")


def measure_one_revolution(side, speed):
    print(f"\n{side.title()} wheel: output-shaft calibration")
    print("Lift the rover, mark the tyre, and keep the wheel visible.")
    input("Press ENTER when the mark is at the top...")
    reset_encoders()
    _drive_wheel(side, abs(speed))
    started = time.monotonic()
    try:
        input("Press ENTER after exactly one complete wheel revolution...\n")
    finally:
        stop()

    elapsed = time.monotonic() - started
    left_ticks, right_ticks = get_encoder_ticks()
    ticks = abs(left_ticks if side == "left" else right_ticks)
    if ticks == 0:
        print("No encoder ticks received; check the signal wiring.")
        return
    print(f"  Measured: {ticks} counts/output-shaft revolution ({elapsed:.1f}s)")
    print("  Set TICKS_PER_WHEEL_REV to this value in helpers/auto_controller.py")


def main():
    parser = argparse.ArgumentParser(description="Calibrate JGB37-520 wheel direction and encoder counts")
    parser.add_argument("--speed", type=float, default=TEST_SPEED, help="PWM duty cycle, 0.05-0.35 (default: 0.20)")
    parser.add_argument("--seconds", type=float, default=TEST_SECONDS, help="Direction test duration (default: 1.5)")
    args = parser.parse_args()
    speed = max(0.05, min(1, args.speed))
    seconds = max(0.2, min(5.0, args.seconds))

    try:
        _require_hardware()
        print("SAFETY: place the rover on a stand so the drive wheels cannot touch the ground.")
        input("Remove obstacles and press ENTER to begin (Ctrl+C also stops safely)...")
        for side in ("left", "right"):
            test_direction(side, speed, seconds)
        for side in ("left", "right"):
            if input(f"Measure one revolution for the {side} wheel? [y/N]: ").strip().lower() == "y":
                measure_one_revolution(side, min(speed, 0.15))
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
    except Exception as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
