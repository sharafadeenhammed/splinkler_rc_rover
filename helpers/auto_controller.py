"""
auto_controller.py  –  Boustrophedon (lawnmower) sweep coverage for the sprinkler rover.

Coverage strategy
─────────────────
The rover sweeps the target area in parallel strips (like mowing a lawn).
Strip width  = rover physical width (default 50 cm) so every pass covers fresh ground.
Strip length = area width (configurable, default 10 m).
Number of strips = ceil(area_height / strip_width).

At the end of each strip the rover pivots 90°, moves one strip-width forward,
pivots 90° again and drives back the other way.  Obstacles interrupt the straight
run and are avoided before the rover resumes the same strip.

All timing-based distance estimates use:
    distance (m) = speed_fraction × FULL_SPEED_MPS × elapsed_seconds

This is an open-loop approximation.  A future encoder/GPS upgrade can replace
the time-based odometry without changing the coverage algorithm structure.
"""

import time
import math
import threading

from helpers.motor_controls  import forward, reverse, turnRight, turnLeft, stop
from helpers.servo_controller import sensor_servo
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.pump_controller  import WATER_PUMP
from helpers.vision_controller import analyze_visual_edges
from helpers.ultrasonic_controller import get_distance_cm

# ─── Thread / mode state ──────────────────────────────────────────────────────
auto_mode_active   = False
auto_thread        = None
auto_lock          = threading.Lock()

# ─── Configurable area & rover parameters ─────────────────────────────────────
# All measurements in metres.
AREA_WIDTH_M       = 10.0   # how far the rover travels on each straight strip
AREA_HEIGHT_M      = 10.0   # total depth of the area to be covered
ROVER_WIDTH_M      = 0.50   # physical rover width  (determines strip spacing)
ROVER_LENGTH_M     = 0.70   # physical rover length (used in turn timing)

# ─── Speed & timing ───────────────────────────────────────────────────────────
AUTO_DRIVE_SPEED   = 0.70   # 0.0 – 1.0  duty-cycle fraction while cruising
FULL_SPEED_MPS     = 0.25   # Approx metres/second at duty-cycle 1.0 on flat ground.
                             # Calibrate once, then it scales with AUTO_DRIVE_SPEED.

# ─── Obstacle avoidance ───────────────────────────────────────────────────────
OBSTACLE_THRESHOLD_CM   = 20.0   # stop if ultrasonic reads ≤ this
use_ultrasonic_in_auto  = True

# ─── Soil / watering ──────────────────────────────────────────────────────────
WATERING_DURATION_SEC   = 3.0    # how long to run pump when dry soil found
SOIL_CHECK_INTERVAL_SEC = 8.0    # check soil every N seconds of driving


# ─── Public setters (called from main.py / WS handler) ────────────────────────
def set_auto_speed(speed_0_to_100: float):
    """Set autonomous cruise speed as a 0–100 percentage from the dashboard."""
    global AUTO_DRIVE_SPEED
    AUTO_DRIVE_SPEED = max(0.1, min(1.0, speed_0_to_100 / 100.0))
    print(f"🤖 Auto speed set to {AUTO_DRIVE_SPEED:.0%}")

def set_area(width_m: float, height_m: float):
    """Set the target coverage area dimensions (metres)."""
    global AREA_WIDTH_M, AREA_HEIGHT_M
    AREA_WIDTH_M  = max(0.5, width_m)
    AREA_HEIGHT_M = max(0.5, height_m)
    print(f"🗺️  Coverage area set to {AREA_WIDTH_M:.1f} m × {AREA_HEIGHT_M:.1f} m")

def set_rover_size(width_m: float, length_m: float):
    """Update rover physical dimensions (affects strip spacing)."""
    global ROVER_WIDTH_M, ROVER_LENGTH_M
    ROVER_WIDTH_M  = max(0.2, width_m)
    ROVER_LENGTH_M = max(0.3, length_m)
    print(f"📐 Rover size updated: {ROVER_WIDTH_M:.2f} m wide × {ROVER_LENGTH_M:.2f} m long")

def enable_ultrasonic_in_auto():
    global use_ultrasonic_in_auto
    use_ultrasonic_in_auto = True
    print("🔊 Ultrasonic sensor ENABLED for autonomous navigation.")
    return True

def disable_ultrasonic_in_auto():
    global use_ultrasonic_in_auto
    use_ultrasonic_in_auto = False
    print("🔇 Ultrasonic sensor DISABLED for autonomous navigation.")
    return False

def toggle_ultrasonic_in_auto():
    return disable_ultrasonic_in_auto() if use_ultrasonic_in_auto else enable_ultrasonic_in_auto()

def is_ultrasonic_enabled_in_auto():
    return use_ultrasonic_in_auto

# ─── Status / progress telemetry (readable from main.py) ─────────────────────
auto_progress = {
    "strip":        0,
    "total_strips": 0,
    "phase":        "idle",     # idle | driving | checking_soil | watering | turning | obstacle
    "coverage_pct": 0.0,
}


# ─────────────────────────────────────────────────────────────────────────────
#  Low-level helpers
# ─────────────────────────────────────────────────────────────────────────────

def _seconds_for_distance(metres: float) -> float:
    """Return how many seconds to drive to cover `metres` at current speed."""
    effective_speed_mps = AUTO_DRIVE_SPEED * FULL_SPEED_MPS
    if effective_speed_mps <= 0:
        return 0.0
    return metres / effective_speed_mps


def _drive_timed(direction_fn, metres: float, check_fn=None) -> bool:
    """
    Drive in `direction_fn` for the time needed to cover `metres`.

    `check_fn` is called every 0.1 s; if it returns True the drive is aborted
    and this function returns False (obstacle / mode-stop).  Returns True on
    successful completion.
    """
    duration = _seconds_for_distance(metres)
    direction_fn(AUTO_DRIVE_SPEED)
    start = time.time()
    while time.time() - start < duration:
        if not auto_mode_active:
            stop()
            return False
        if check_fn and check_fn():
            stop()
            return False
        time.sleep(0.05)
    stop()
    return True


def _pivot_90(clockwise: bool):
    """Turn 90° in-place using time-based estimation."""
    # Approximation: at full pivot speed, a 90° turn takes ~0.6 s for this wheelbase.
    # Scale by rover width.  Adjust TURN_90_SEC_BASE if actual turn is over/under 90°.
    TURN_90_SEC_BASE = 0.65   # seconds at speed 1.0 for a 90° pivot on this chassis
    duration = TURN_90_SEC_BASE / AUTO_DRIVE_SPEED
    if clockwise:
        turnRight(AUTO_DRIVE_SPEED)
    else:
        turnLeft(AUTO_DRIVE_SPEED)
    start = time.time()
    while time.time() - start < duration:
        if not auto_mode_active:
            stop()
            return
        time.sleep(0.05)
    stop()
    time.sleep(0.1)


def _obstacle_check() -> bool:
    """Return True if an obstacle is detected (ultrasonic or vision)."""
    if use_ultrasonic_in_auto:
        dist = get_distance_cm()
        if dist is not None and 0 < dist <= OBSTACLE_THRESHOLD_CM:
            return True
    vis_state, _, _ = analyze_visual_edges()
    return vis_state != "CLEAR"


def _evade_obstacle():
    """Reverse slightly then turn away from the detected obstacle side."""
    print("🚨 Obstacle! Executing evasive manoeuvre...")
    auto_progress["phase"] = "obstacle"
    stop()
    time.sleep(0.15)

    # Back up half the rover length
    reverse(AUTO_DRIVE_SPEED)
    time.sleep(_seconds_for_distance(ROVER_LENGTH_M * 0.5))
    stop()
    time.sleep(0.1)

    # Choose turn direction from vision
    vis_state, _, _ = analyze_visual_edges()
    if vis_state == "OBSTACLE_LEFT":
        print("↪️  Turning RIGHT to avoid left obstacle.")
        _pivot_90(clockwise=True)
    else:
        print("↩️  Turning LEFT to avoid right obstacle.")
        _pivot_90(clockwise=False)

    # Drive forward slightly past the obstacle before attempting to resume
    forward(AUTO_DRIVE_SPEED)
    time.sleep(_seconds_for_distance(ROVER_LENGTH_M * 0.8))
    stop()
    time.sleep(0.15)


def _check_and_water():
    """Deploy sensor probe, read moisture, pump if dry, retract."""
    global auto_progress
    print("🌱 Halting for soil moisture check...")
    auto_progress["phase"] = "checking_soil"
    stop()
    WATER_PUMP.turnOffPump()
    time.sleep(0.3)

    sensor_servo.angle = -20     # deploy probe
    time.sleep(1.2)              # settle in soil

    soil_state = MOISTURE_SENSOR.getSoilMoistureState()
    print(f"🌱 Soil: {soil_state}")

    if str(soil_state).strip().lower() == "dry":
        auto_progress["phase"] = "watering"
        print(f"💧 DRY – pumping for {WATERING_DURATION_SEC} s...")
        WATER_PUMP.turnOnPump()
        pump_start = time.time()
        while (time.time() - pump_start < WATERING_DURATION_SEC) and auto_mode_active:
            time.sleep(0.1)
        WATER_PUMP.turnOffPump()
        print("💧 Watering done.")
        time.sleep(0.3)
    else:
        print("✅ Soil WET – skipping pump.")
        WATER_PUMP.turnOffPump()

    sensor_servo.angle = -90     # retract probe
    time.sleep(0.8)


# ─────────────────────────────────────────────────────────────────────────────
#  Boustrophedon sweep  (the main autonomous routine)
# ─────────────────────────────────────────────────────────────────────────────

def autonomous_routine():
    """
    Lawnmower-pattern area coverage with soil sensing and obstacle avoidance.

    Coordinate frame (rover starts at origin facing positive X):

        ┌──────────────────────── AREA_WIDTH_M ───────────────────────┐
        │  →→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→  strip 0       │
        │  ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←  strip 1       │ AREA
        │  →→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→  strip 2       │ HEIGHT
        │  ...                                                         │ _M
        └──────────────────────────────────────────────────────────────┘
    """
    global auto_mode_active, auto_progress

    total_strips = max(1, math.ceil(AREA_HEIGHT_M / ROVER_WIDTH_M))
    auto_progress["total_strips"] = total_strips
    auto_progress["strip"]        = 0
    auto_progress["phase"]        = "driving"
    auto_progress["coverage_pct"] = 0.0

    print(f"🤖 Autonomous sweep START  –  area {AREA_WIDTH_M:.1f} m × {AREA_HEIGHT_M:.1f} m,  "
          f"{total_strips} strips,  speed {AUTO_DRIVE_SPEED:.0%}")

    # ── Even strips go left→right (clockwise turn between strips)
    # ── Odd strips go right→left (anti-clockwise turn between strips)
    going_right = True
    last_soil_check = time.time()

    for strip in range(total_strips):
        if not auto_mode_active:
            break

        auto_progress["strip"]        = strip + 1
        auto_progress["coverage_pct"] = round((strip / total_strips) * 100, 1)
        print(f"🗺️  Strip {strip + 1}/{total_strips}  {'→' if going_right else '←'}")
        auto_progress["phase"] = "driving"

        # ── Drive the full strip length, checking for obstacles on the way ──
        strip_start = time.time()
        distance_covered = 0.0
        drive_start = time.time()

        while auto_mode_active:
            if _obstacle_check():
                _evade_obstacle()
                if not auto_mode_active:
                    break
                # Resume strip; recalculate remaining distance
                elapsed_before = time.time() - drive_start
                distance_covered += AUTO_DRIVE_SPEED * FULL_SPEED_MPS * elapsed_before
                distance_covered = min(distance_covered, AREA_WIDTH_M)
                drive_start = time.time()
                auto_progress["phase"] = "driving"
                continue

            # Soil check timer
            now = time.time()
            if now - last_soil_check >= SOIL_CHECK_INTERVAL_SEC:
                elapsed_before = time.time() - drive_start
                distance_covered += AUTO_DRIVE_SPEED * FULL_SPEED_MPS * elapsed_before
                _check_and_water()
                last_soil_check = time.time()
                drive_start = time.time()
                auto_progress["phase"] = "driving"
                if not auto_mode_active:
                    break
                continue

            # Normal forward movement
            forward(AUTO_DRIVE_SPEED)
            time.sleep(0.05)
            distance_covered += AUTO_DRIVE_SPEED * FULL_SPEED_MPS * 0.05

            if distance_covered >= AREA_WIDTH_M:
                stop()
                break

        if not auto_mode_active:
            break

        # ── End-of-strip: final soil check ───────────────────────────────────
        _check_and_water()
        last_soil_check = time.time()

        if not auto_mode_active:
            break

        # ── Last strip – no need to turn ─────────────────────────────────────
        if strip == total_strips - 1:
            break

        # ── Turn 90°, advance one strip width, turn 90° back ─────────────────
        auto_progress["phase"] = "turning"
        print(f"↕️  End-of-strip turn (strip {strip + 1} → {strip + 2})")

        if going_right:
            # Turn clockwise (right), advance, turn clockwise again → now facing left
            _pivot_90(clockwise=True)
            if not auto_mode_active: break
            _drive_timed(forward, ROVER_WIDTH_M)
            if not auto_mode_active: break
            _pivot_90(clockwise=True)
        else:
            # Turn anti-clockwise (left), advance, turn anti-clockwise again → now facing right
            _pivot_90(clockwise=False)
            if not auto_mode_active: break
            _drive_timed(forward, ROVER_WIDTH_M)
            if not auto_mode_active: break
            _pivot_90(clockwise=False)

        going_right = not going_right

    # ── Coverage complete ─────────────────────────────────────────────────────
    stop()
    WATER_PUMP.turnOffPump()
    sensor_servo.angle = -90
    auto_progress["phase"]        = "idle"
    auto_progress["coverage_pct"] = 100.0 if auto_mode_active else auto_progress["coverage_pct"]
    auto_mode_active = False
    print("✅ Autonomous area sweep COMPLETE.")


# ─────────────────────────────────────────────────────────────────────────────
#  Public mode controls
# ─────────────────────────────────────────────────────────────────────────────

def start_auto_mode():
    global auto_mode_active, auto_thread
    with auto_lock:
        if not auto_mode_active:
            auto_mode_active = True
            auto_thread = threading.Thread(target=autonomous_routine, daemon=True)
            auto_thread.start()
            return True
    return auto_mode_active


def stop_auto_mode():
    global auto_mode_active
    with auto_lock:
        auto_mode_active = False
        stop()
        WATER_PUMP.turnOffPump()
        sensor_servo.angle = -90
    auto_progress["phase"] = "idle"
    return False


def toggle_auto_mode():
    if auto_mode_active:
        return stop_auto_mode()
    return start_auto_mode()


def is_auto_active():
    return auto_mode_active


def get_auto_progress():
    return dict(auto_progress)


__all__ = [
    "start_auto_mode",
    "stop_auto_mode",
    "toggle_auto_mode",
    "is_auto_active",
    "get_auto_progress",
    "set_auto_speed",
    "set_area",
    "set_rover_size",
    "enable_ultrasonic_in_auto",
    "disable_ultrasonic_in_auto",
    "toggle_ultrasonic_in_auto",
    "is_ultrasonic_enabled_in_auto",
    "AUTO_DRIVE_SPEED",
    "AREA_WIDTH_M",
    "AREA_HEIGHT_M",
    "ROVER_WIDTH_M",
    "ROVER_LENGTH_M",
]
