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

Navigation uses wheel encoders for precise distance tracking and PID-controlled
straight-line correction.  The rover measures ticks on the left and right wheels
to know exactly how far it has gone, and adjusts motor power to keep the wheels
synchronised (preventing drift).
"""

import time
import math
import threading

from helpers.motor_controls      import forward, reverse, turnRight, turnLeft, stop, drive_tank
from helpers.servo_controller    import sensor_servo
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.pump_controller     import WATER_PUMP
from helpers.ultrasonic_controller import get_distance_cm
from helpers.encoder_controller  import (
    get_encoder_ticks, reset_encoders, LEFT_ENCODER, RIGHT_ENCODER,
    COUNTS_PER_OUTPUT_REV,
)
from helpers.mpu6050 import get_imu_snapshot, reset_imu_heading

# ─── Thread / mode state ──────────────────────────────────────────────────────
auto_mode_active   = False
auto_thread        = None
auto_lock          = threading.Lock()

# ─── Configurable area & rover parameters ─────────────────────────────────────
# All measurements in metres.
AREA_WIDTH_M       = 10.0   # how far the rover travels on each straight strip
AREA_HEIGHT_M      = 10.0   # total depth of the area to be covered
ROVER_WIDTH_M      = 0.30   # physical rover width  (determines strip spacing)
ROVER_LENGTH_M     = 0.70   # physical rover length
AREA_POLYGON       = []     # optional local-coordinate boundary [[x, y], ...]

# ─── Speed ────────────────────────────────────────────────────────────────────
AUTO_DRIVE_SPEED   = 0.70   # 0.0 – 1.0  duty-cycle fraction while cruising

# ─── Encoder calibration ──────────────────────────────────────────────────────
# JGB37-520 motor specs:
#   - 11 magnetic pulses per motor revolution (before gearbox)
#   - Gear ratio varies by model.  Common ratios: 1:30, 1:56, 1:90, 1:210
#   - We read RISING_EDGE on channel A only → 11 ticks per motor rev
#   - Output shaft counts per revolution = 11 × gear_ratio
#
# ──── YOU MUST CALIBRATE THIS ────
# Mark a point on the wheel, reset the encoder, push the rover forward
# until the wheel completes exactly 1 full revolution, then read the tick
# count.  Enter that number below.
TICKS_PER_WHEEL_REV = COUNTS_PER_OUTPUT_REV

# Wheel circumference in metres.  Measure outer diameter of the rubber tyre.
WHEEL_DIAMETER_M    = 0.065  # 65 mm is typical for the included wheels
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

# Derived: how far the rover moves per encoder tick
METRES_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV

# Wheel track: distance between left wheel centre and right wheel centre.
# For a pivot turn, 90° arc length = (π/2) × (track/2) per wheel.
WHEEL_TRACK_M = 0.30   # 30 cm between wheel centres.  CALIBRATE THIS!
TICKS_PER_90_TURN = int((math.pi / 2.0) * (WHEEL_TRACK_M / 2.0) / METRES_PER_TICK)

# ─── PID straight-line correction ─────────────────────────────────────────────
PID_KP = 0.005   # proportional gain: how aggressively to correct drift
PID_KI = 0.0001  # integral gain:     correct persistent slight bias
PID_KD = 0.002   # derivative gain:   dampen oscillations
IMU_HEADING_KP = 0.35  # duty-cycle correction per radian of heading drift

# ─── Obstacle avoidance (ultrasonic only — the camera vision pipeline was
# removed: it was the biggest CPU/memory consumer and correlated with the
# board's power collapses) ────────────────────────────────────────────────────
OBSTACLE_THRESHOLD_CM   = 20.0
# A single ultrasonic sample can glitch (stale async reading, stray echo);
# require this many consecutive short readings before acting on an obstacle.
# Pivot turns are expensive to make on a phantom.
OBSTACLE_CONFIRM_READINGS = 2
use_ultrasonic_in_auto  = True
_obstacle_hits = 0
# Alternating evasive direction so repeated obstacles cannot trap the rover
# in a one-sided turn loop.
_evasion_side = "left"

# ─── Soil / watering ──────────────────────────────────────────────────────────
# When the probe reads DRY the pump starts AND the rover keeps moving, so the
# water wets the ground along the rover's path instead of pooling in one spot.
WATERING_RUN_M          = 1.0   # metres driven during one dry-soil watering run
SOIL_CHECK_INTERVAL_M   = 1.5   # check soil every 1.5 metres of forward travel


# ─── Public setters (called from main.py / WS handler) ────────────────────────

def set_auto_speed(speed_0_to_100: float):
    """Set autonomous cruise speed as a 0–100 percentage from the dashboard."""
    global AUTO_DRIVE_SPEED
    AUTO_DRIVE_SPEED = max(0.1, min(1.0, speed_0_to_100 / 100.0))
    print(f"🤖 Auto speed set to {AUTO_DRIVE_SPEED:.0%}")

def set_area(width_m: float, height_m: float):
    """Set the target coverage area dimensions (metres)."""
    global AREA_WIDTH_M, AREA_HEIGHT_M, AREA_POLYGON
    AREA_WIDTH_M  = max(0.5, width_m)
    AREA_HEIGHT_M = max(0.5, height_m)
    AREA_POLYGON = [[0.0, 0.0], [AREA_WIDTH_M, 0.0], [AREA_WIDTH_M, AREA_HEIGHT_M], [0.0, AREA_HEIGHT_M]]
    print(f"🗺️  Coverage area set to {AREA_WIDTH_M:.1f} m × {AREA_HEIGHT_M:.1f} m")


def set_area_polygon(points):
    """Load a saved map polygon for autonomous coverage."""
    global AREA_WIDTH_M, AREA_HEIGHT_M, AREA_POLYGON
    polygon = [[float(point[0]), float(point[1])] for point in points]
    if len(polygon) < 3:
        raise ValueError("A polygon needs at least three points")
    AREA_WIDTH_M = max(x for x, _ in polygon) - min(x for x, _ in polygon)
    AREA_HEIGHT_M = max(y for _, y in polygon) - min(y for _, y in polygon)
    if AREA_WIDTH_M <= 0 or AREA_HEIGHT_M <= 0:
        raise ValueError("The polygon must have width and height")
    AREA_POLYGON = polygon

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


# ─── Status / progress telemetry ─────────────────────────────────────────────
auto_progress = {
    "strip":        0,
    "total_strips": 0,
    "phase":        "idle",
    "coverage_pct": 0.0,
    "left_ticks":   0,
    "right_ticks":  0,
}


# ─────────────────────────────────────────────────────────────────────────────
#  Encoder helpers
# ─────────────────────────────────────────────────────────────────────────────

def _has_encoders() -> bool:
    """Return True if both hardware encoders were initialised."""
    return LEFT_ENCODER is not None and RIGHT_ENCODER is not None


def _ticks_for_distance(metres: float) -> int:
    """Return how many encoder ticks equal the given distance."""
    if METRES_PER_TICK <= 0:
        return 0
    return int(metres / METRES_PER_TICK)


def _avg_ticks() -> float:
    """Average of absolute left + right ticks since last reset."""
    lt, rt = get_encoder_ticks()
    return (abs(lt) + abs(rt)) / 2.0


def _update_tick_telemetry():
    """Push live tick counts into the progress dict for the dashboard."""
    lt, rt = get_encoder_ticks()
    auto_progress["left_ticks"]  = lt
    auto_progress["right_ticks"] = rt


def _coverage_segments():
    """Return horizontal scan-line widths that fall inside the boundary."""
    if len(AREA_POLYGON) < 3:
        return [(AREA_WIDTH_M, 0.0)] * max(1, math.ceil(AREA_HEIGHT_M / ROVER_WIDTH_M))
    min_y = min(point[1] for point in AREA_POLYGON)
    max_y = max(point[1] for point in AREA_POLYGON)
    segments = []
    y = min_y + ROVER_WIDTH_M / 2.0
    while y < max_y:
        intersections = []
        for index, first in enumerate(AREA_POLYGON):
            second = AREA_POLYGON[(index + 1) % len(AREA_POLYGON)]
            if (first[1] <= y < second[1]) or (second[1] <= y < first[1]):
                if second[1] != first[1]:
                    x = first[0] + (y - first[1]) * (second[0] - first[0]) / (second[1] - first[1])
                    intersections.append(x)
        intersections.sort()
        for index in range(0, len(intersections) - 1, 2):
            width = intersections[index + 1] - intersections[index]
            if width > 0.10:
                segments.append((width, y))
        y += ROVER_WIDTH_M
    return segments or [(AREA_WIDTH_M, min_y)]


# ─────────────────────────────────────────────────────────────────────────────
#  PID Straight-Line Drive  (the core improvement)
# ─────────────────────────────────────────────────────────────────────────────

def _drive_straight(target_metres: float, check_obstacle=True,
                    soil_check_fn=None) -> float:
    """
    Drive forward in a straight line for `target_metres` using encoder
    feedback and PID correction.

    Returns the distance actually covered (metres).

    If `check_obstacle` is True, the drive will pause and return early when
    an obstacle is detected (caller handles evasion).

    If `soil_check_fn` is provided, it is called with the distance covered
    so far; it returns True if the rover should stop for a soil check.
    """
    if not _has_encoders():
        # Graceful fallback to time-based driving if encoders failed
        return _drive_straight_timed(target_metres, check_obstacle)

    target_ticks = _ticks_for_distance(target_metres)
    soil_check_ticks = _ticks_for_distance(SOIL_CHECK_INTERVAL_M)

    reset_encoders()
    ticks_since_soil_check = 0

    # PID state
    prev_error = 0
    integral   = 0

    base_speed = AUTO_DRIVE_SPEED
    target_heading = get_imu_snapshot().get("heading_rad", 0.0)

    while auto_mode_active:
        lt, rt = get_encoder_ticks()
        avg    = (abs(lt) + abs(rt)) / 2.0
        _update_tick_telemetry()

        # ── Distance target reached ──────────────────────────────────────
        if avg >= target_ticks:
            stop()
            return avg * METRES_PER_TICK

        # ── Obstacle check ───────────────────────────────────────────────
        if check_obstacle and _obstacle_check():
            stop()
            return avg * METRES_PER_TICK   # caller will handle evasion

        # ── Soil check interval (distance-based, not time-based) ─────────
        ticks_since_soil_check = avg  # simplification: reset happens externally
        if soil_check_fn and avg > 0 and (avg % soil_check_ticks < 2):
            dist_so_far = avg * METRES_PER_TICK
            if soil_check_fn(dist_so_far):
                stop()
                return dist_so_far

        # ── PID correction ───────────────────────────────────────────────
        # Error = left ticks minus right ticks.
        # If left > right → rover drifting right → slow left / speed right
        error = abs(lt) - abs(rt)
        integral += error
        derivative = error - prev_error
        prev_error = error

        correction = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative)
        imu = get_imu_snapshot()
        if imu.get("available"):
            heading_error = (imu["heading_rad"] - target_heading + math.pi) % (2 * math.pi) - math.pi
            correction += IMU_HEADING_KP * heading_error

        left_speed  = base_speed - correction
        right_speed = base_speed + correction

        # Clamp speeds
        left_speed  = max(0.15, min(1.0, left_speed))
        right_speed = max(0.15, min(1.0, right_speed))

        drive_tank(left_speed, right_speed)
        time.sleep(0.02)  # 50 Hz control loop

    stop()
    lt, rt = get_encoder_ticks()
    return ((abs(lt) + abs(rt)) / 2.0) * METRES_PER_TICK


def _drive_straight_timed(target_metres: float, check_obstacle=True) -> float:
    """Fallback: time-based forward drive if encoders are unavailable."""
    FULL_SPEED_MPS = 0.25
    effective_speed_mps = AUTO_DRIVE_SPEED * FULL_SPEED_MPS
    if effective_speed_mps <= 0:
        return 0.0
    duration = target_metres / effective_speed_mps
    forward(AUTO_DRIVE_SPEED)
    start = time.time()
    while time.time() - start < duration:
        if not auto_mode_active:
            stop()
            elapsed = time.time() - start
            return elapsed * effective_speed_mps
        if check_obstacle and _obstacle_check():
            stop()
            elapsed = time.time() - start
            return elapsed * effective_speed_mps
        time.sleep(0.05)
    stop()
    return target_metres


# ─────────────────────────────────────────────────────────────────────────────
#  Encoder-based 90° pivot turn
# ─────────────────────────────────────────────────────────────────────────────

def _pivot_90(clockwise: bool):
    """Turn exactly 90° by counting encoder ticks on both wheels."""
    if not _has_encoders():
        _pivot_90_timed(clockwise)
        return

    target_ticks = TICKS_PER_90_TURN
    reset_encoders()

    turn_speed = min(AUTO_DRIVE_SPEED, 0.6)   # turns work better at moderate speed

    if clockwise:
        turnRight(turn_speed)
    else:
        turnLeft(turn_speed)

    while auto_mode_active:
        lt, rt = get_encoder_ticks()
        avg = (abs(lt) + abs(rt)) / 2.0
        if avg >= target_ticks:
            break
        time.sleep(0.01)

    stop()
    time.sleep(0.1)


def _pivot_90_timed(clockwise: bool):
    """Fallback: time-based 90° turn."""
    TURN_90_SEC_BASE = 0.65
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


# ─────────────────────────────────────────────────────────────────────────────
#  Encoder-based precise distance drive (for strip-width advances)
# ─────────────────────────────────────────────────────────────────────────────

def _drive_distance(metres: float, check_fn=None) -> bool:
    """
    Drive forward exactly `metres` using encoder counting.
    Returns True if the full distance was completed.
    """
    if not _has_encoders():
        # Time-based fallback
        FULL_SPEED_MPS = 0.25
        effective_speed_mps = AUTO_DRIVE_SPEED * FULL_SPEED_MPS
        if effective_speed_mps <= 0:
            return True
        duration = metres / effective_speed_mps
        forward(AUTO_DRIVE_SPEED)
        start = time.time()
        while time.time() - start < duration:
            if not auto_mode_active:
                stop()
                return False
            time.sleep(0.05)
        stop()
        return True

    target_ticks = _ticks_for_distance(metres)
    reset_encoders()

    # Use PID for straight advance too
    prev_error = 0
    integral   = 0
    base_speed = AUTO_DRIVE_SPEED
    target_heading = get_imu_snapshot().get("heading_rad", 0.0)

    while auto_mode_active:
        lt, rt = get_encoder_ticks()
        avg = (abs(lt) + abs(rt)) / 2.0
        if avg >= target_ticks:
            break

        error = abs(lt) - abs(rt)
        integral += error
        derivative = error - prev_error
        prev_error = error
        correction = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative)
        imu = get_imu_snapshot()
        if imu.get("available"):
            heading_error = (imu["heading_rad"] - target_heading + math.pi) % (2 * math.pi) - math.pi
            correction += IMU_HEADING_KP * heading_error

        left_speed  = max(0.15, min(1.0, base_speed - correction))
        right_speed = max(0.15, min(1.0, base_speed + correction))
        drive_tank(left_speed, right_speed)
        time.sleep(0.02)

    stop()
    return auto_mode_active


# ─────────────────────────────────────────────────────────────────────────────
#  Obstacle & Watering helpers (unchanged logic)
# ─────────────────────────────────────────────────────────────────────────────

def _obstacle_check() -> bool:
    """True when the ultrasonic reports a barrier ahead (debounced).

    Camera vision was removed, so the ultrasonic is now the only obstacle
    sense.  A fresh read happens on every call, so one clear reading resets
    the counter immediately.
    """
    global _obstacle_hits
    if not use_ultrasonic_in_auto:
        _obstacle_hits = 0
        return False
    dist = get_distance_cm()
    if dist is not None and 0 < dist <= OBSTACLE_THRESHOLD_CM:
        _obstacle_hits += 1
    else:
        _obstacle_hits = 0
    return _obstacle_hits >= OBSTACLE_CONFIRM_READINGS


def _evade_obstacle():
    """Reverse slightly then turn away from the detected obstacle side."""
    global _evasion_side
    print("🚨 Obstacle! Executing evasive manoeuvre...")
    auto_progress["phase"] = "obstacle"
    stop()
    time.sleep(0.15)

    # Back up half the rover length using encoder counting
    backup_ticks = _ticks_for_distance(ROVER_LENGTH_M * 0.5)
    if _has_encoders():
        reset_encoders()
        reverse(AUTO_DRIVE_SPEED)
        while auto_mode_active:
            lt, rt = get_encoder_ticks()
            if (abs(lt) + abs(rt)) / 2.0 >= backup_ticks:
                break
            time.sleep(0.02)
        stop()
    else:
        FULL_SPEED_MPS = 0.25
        reverse(AUTO_DRIVE_SPEED)
        time.sleep((ROVER_LENGTH_M * 0.5) / (AUTO_DRIVE_SPEED * FULL_SPEED_MPS))
        stop()

    time.sleep(0.1)

    # Pick a side (alternating between evasions — no vision to tell us which
    # way is clearer), verify the chosen way is actually open, then drive
    # past the obstacle.
    clockwise = _evasion_side == "right"
    _evasion_side = "right" if _evasion_side == "left" else "left"
    print("↪️  Turning RIGHT to evade." if clockwise else "↩️  Turning LEFT to evade.")
    _pivot_90(clockwise=clockwise)
    final_turn_clockwise = clockwise
    if _obstacle_check():
        print("↩️  Chosen side blocked – swinging 180°.")
        _pivot_90(clockwise=not clockwise)
        _pivot_90(clockwise=not clockwise)
        final_turn_clockwise = not clockwise

    # Drive forward past the obstacle, then undo the evasion turn so the
    # rover resumes the strip heading it had before: the boustrophedon sweep
    # depends on its passes staying strictly 180° apart.
    _drive_distance(ROVER_LENGTH_M * 0.8)
    _pivot_90(clockwise=not final_turn_clockwise)
    time.sleep(0.15)


def _water_while_moving(distance_m: float) -> float:
    """Pump ON while driving forward so the water wets the ground along the
    rover's path instead of pooling at one spot.

    Returns the metres actually covered.  The pump is cut on every exit path
    (obstacle, stall, auto-mode stop, exception) so it can never be left
    running behind an error.
    """
    global _obstacle_hits
    remaining = distance_m
    covered_total = 0.0
    stalls = 0
    WATER_PUMP.turnOnPump()
    try:
        while remaining > 0.05 and auto_mode_active:
            covered = _drive_straight(min(remaining, 0.4), check_obstacle=True)
            remaining -= covered
            covered_total += covered
            if not auto_mode_active:
                break
            if _obstacle_check():
                WATER_PUMP.turnOffPump()       # never spray a wall
                _obstacle_hits = 0
                _evade_obstacle()
                if auto_mode_active:
                    WATER_PUMP.turnOnPump()
                stalls = 0
            elif covered <= 0.02:
                stalls += 1
                if stalls >= 2:
                    print("⚠️ Watering run stalled – stopping pump, resuming sweep.")
                    break
            else:
                stalls = 0
            time.sleep(0.05)
    finally:
        WATER_PUMP.turnOffPump()
    return covered_total


def _check_and_water() -> float:
    """Deploy the soil probe and read moisture.  When DRY, start the pump
    and drive the next stretch so the water spreads along the rover path.

    Returns the metres advanced during the watering run, so the caller can
    fold it back into the strip accounting.
    """
    print("🌱 Halting for soil moisture check...")
    auto_progress["phase"] = "checking_soil"
    stop()
    WATER_PUMP.turnOffPump()
    time.sleep(0.3)

    sensor_servo.angle = -20     # deploy probe
    time.sleep(1.2)

    soil_state = MOISTURE_SENSOR.getSoilMoistureState()
    print(f"🌱 Soil: {soil_state}")

    watered_m = 0.0
    if str(soil_state).strip().lower() == "dry":
        auto_progress["phase"] = "watering"
        print(f"💧 DRY – pumping while advancing {WATERING_RUN_M:.1f} m...")
        sensor_servo.angle = -90     # retract the probe before moving
        time.sleep(0.8)
        watered_m = _water_while_moving(WATERING_RUN_M)
        print(f"💧 Watering run done ({watered_m:.2f} m).")
    else:
        print("✅ Soil WET – skipping pump.")
        WATER_PUMP.turnOffPump()

    sensor_servo.angle = -90     # retract probe (no-op if already home)
    time.sleep(0.8)
    auto_progress["phase"] = "driving"
    return watered_m


# ─────────────────────────────────────────────────────────────────────────────
#  Boustrophedon sweep  (main autonomous routine)
# ─────────────────────────────────────────────────────────────────────────────

def autonomous_routine():
    """
    Lawnmower-pattern area coverage with:
      • Encoder-based PID straight-line driving
      • Encoder-based precise 90° turns
      • Distance-based soil moisture checks (pump-on-while-driving when dry)
      • Debounced ultrasonic obstacle avoidance with heading re-alignment

        ┌──────────────────────── AREA_WIDTH_M ───────────────────────┐
        │  →→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→  strip 0       │
        │  ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←  strip 1       │ AREA
        │  →→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→  strip 2       │ HEIGHT
        │  ...                                                         │ _M
        └──────────────────────────────────────────────────────────────┘
    """
    global auto_mode_active, auto_progress

    coverage_segments = _coverage_segments()
    total_strips = len(coverage_segments)
    auto_progress["total_strips"] = total_strips
    auto_progress["strip"]        = 0
    auto_progress["phase"]        = "driving"
    auto_progress["coverage_pct"] = 0.0

    enc_status = "ENCODERS ✅" if _has_encoders() else "TIME-BASED ⏱️ (no encoders)"
    print(f"🤖 Autonomous sweep START  –  area {AREA_WIDTH_M:.1f} m × {AREA_HEIGHT_M:.1f} m,  "
          f"{total_strips} strips,  speed {AUTO_DRIVE_SPEED:.0%},  nav: {enc_status}")

    going_right = True
    distance_since_soil_check = 0.0

    for strip in range(total_strips):
        if not auto_mode_active:
            break

        auto_progress["strip"]        = strip + 1
        auto_progress["coverage_pct"] = round((strip / total_strips) * 100, 1)
        print(f"🗺️  Strip {strip + 1}/{total_strips}  {'→' if going_right else '←'}")
        auto_progress["phase"] = "driving"

        # ── Drive the full strip, handling obstacles along the way ────────
        segment_width, _segment_y = coverage_segments[strip]
        remaining_m = segment_width
        distance_since_soil_check = 0.0
        stalls = 0
        stall_backoffs = 0

        while auto_mode_active and remaining_m > 0.01:
            # Decide how far to drive before next soil check
            drive_target = min(remaining_m, SOIL_CHECK_INTERVAL_M - distance_since_soil_check)
            drive_target = max(drive_target, 0.05)  # minimum move

            covered = _drive_straight(drive_target, check_obstacle=True)
            remaining_m -= covered
            distance_since_soil_check += covered
            _update_tick_telemetry()

            if not auto_mode_active:
                break

            # Did we stop because of an obstacle?
            if _obstacle_check():
                _evade_obstacle()
                if not auto_mode_active:
                    break
                auto_progress["phase"] = "driving"
                continue

            # Progress guard: repeated zero-progress stretches with a clear
            # path ahead mean the wheels are not turning (wedged or a motor
            # gave up).  Back off and retry; give up cleanly rather than
            # spinning in this loop forever.
            if covered <= 0.02:
                stalls += 1
            else:
                stalls = 0
            if stalls >= 3:
                stalls = 0
                stall_backoffs += 1
                if stall_backoffs > 3:
                    print("⚠️ Rover is stuck – no progress after 3 back-offs; stopping auto mode.")
                    stop_auto_mode()
                    break
                print(f"⚠️ No forward progress – backing off ({stall_backoffs}/3).")
                auto_progress["phase"] = "obstacle"
                reverse(AUTO_DRIVE_SPEED)
                time.sleep(0.5)
                stop()
                continue

            # Soil check due?  The watering run moves the rover forward, so
            # its distance is folded back into the strip accounting.
            if distance_since_soil_check >= SOIL_CHECK_INTERVAL_M:
                watered_m = _check_and_water()
                remaining_m = max(0.0, remaining_m - watered_m)
                distance_since_soil_check = watered_m
                if not auto_mode_active:
                    break
                auto_progress["phase"] = "driving"

        if not auto_mode_active:
            break

        # ── End-of-strip: final soil check ───────────────────────────────
        _check_and_water()

        if not auto_mode_active:
            break

        # ── Last strip – no turn needed ──────────────────────────────────
        if strip == total_strips - 1:
            break

        # ── Turn 90°, advance one strip width, turn 90° ──────────────────
        auto_progress["phase"] = "turning"
        print(f"↕️  End-of-strip turn (strip {strip + 1} → {strip + 2})")

        if going_right:
            _pivot_90(clockwise=True)
            if not auto_mode_active: break
            _drive_distance(ROVER_WIDTH_M)
            if not auto_mode_active: break
            _pivot_90(clockwise=True)
        else:
            _pivot_90(clockwise=False)
            if not auto_mode_active: break
            _drive_distance(ROVER_WIDTH_M)
            if not auto_mode_active: break
            _pivot_90(clockwise=False)

        going_right = not going_right

    # ── Coverage complete ─────────────────────────────────────────────────
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
            reset_imu_heading()
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
    _update_tick_telemetry()
    return dict(auto_progress)

__all__ = [
    "start_auto_mode",
    "stop_auto_mode",
    "toggle_auto_mode",
    "is_auto_active",
    "get_auto_progress",
    "set_auto_speed",
    "set_area",
    "set_area_polygon",
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
