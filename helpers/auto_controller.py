"""Autonomous area coverage for the sprinkler rover.

The mission is one explicit sweep of a work area the dashboard defines: a
breadth and a length in metres, centred on the rover.  The number of passes is
the breadth divided by the width of the machine's work band, and the passes
run along the length, alternating direction (boustrophedon):

    transit to the area -> for every pass: line up, drive, step across

Design rules this rewrite follows, learned from field runs where the rover
drove a short way and quietly stopped:

* The mission never ends because one strip did not work.  A strip that times
  out, stalls, or is blocked is recorded as incomplete and the mission moves
  on, so the progress bar always advances and the reason for a skip is
  visible in telemetry.
* Only a genuinely unrecoverable error (mission time limit, an exception in
  the engine, or the user pressing stop) sets a fault.
* No fixed duration drives anything: every distance comes from the fused pose
  (or the encoders as a fallback) and every turn is closed on the gyro.
* Nothing is commanded below MIN_DUTY - under it the gearmotors hum without
  turning - and the straight-line base duty sits above it so both wheels can
  still be steered up or down.

Planning lives in helpers/auto_plan.py and stays pure so it can be tested on
a machine without the rover hardware (test_auto_plan.py).
"""

import math
import threading
import time

from helpers.auto_plan import (
    DEFAULT_SWATH_M,
    MIN_AREA_M,
    MIN_DUTY,
    plan_rectangle_boustrophedon,
    plan_connector,
    segment_heading,
    segment_length,
    steer_duties,
    sweep_summary,
    total_planned_length,
    track_errors,
    wrap_angle,
)
from helpers.encoder_controller import TICKS_PER_WHEEL_REV, get_encoder_ticks
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.motor_controls import drive_tank, reverse, stop, turnLeft, turnRight
from helpers.mpu6050 import get_imu_snapshot
from helpers.pump_controller import WATER_PUMP
from helpers.servo_controller import sensor_servo
from helpers.ultrasonic_controller import get_distance_cm

# ---------------------------------------------------------------- geometry ---
WHEEL_DIAMETER_M = 0.065
WHEEL_TRACK_M = 0.30
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
METRES_PER_TICK = WHEEL_CIRCUMFERENCE_M / max(1, TICKS_PER_WHEEL_REV)

ROVER_WIDTH_M = 0.30
ROVER_LENGTH_M = 0.70

# ------------------------------------------------------------- work area ----
# The area is a rectangle in the dashboard's local frame (x east, y north,
# metres from the saved map origin): breadth east-west, length north-south.
# It is centred on the rover where it stood when the area was set; nothing is
# planned until one exists.
AREA_CENTER_X = None
AREA_CENTER_Y = None
AREA_WIDTH_M = 0.0            # east-west side
AREA_HEIGHT_M = 0.0           # north-south side
MAX_AREA_SIDE_M = 100.0

# Width of ground one pass covers - the device's work band, not the chassis.
# The number of passes is the area breadth divided by this.
SWATH_M = DEFAULT_SWATH_M

# ---------------------------------------------------------------- driving ----
# MIN_DUTY (auto_plan) is the hard floor: below it the gearmotors hum without
# turning.  STEER_HEADROOM lifts the straight-line base duty off that floor so
# a wheel can be sped up or slowed down inside the usable MIN_DUTY..1.0 band.
STEER_HEADROOM = 0.10
AUTO_DRIVE_SPEED = 0.90           # base duty while following a strip
TURN_DUTY = 0.90                  # pivots need more torque than rolling

LOOP_PERIOD_S = 0.02
TURN_TOLERANCE_RAD = math.radians(4.0)
TURN_SETTLE_S = 0.15
POINT_TOLERANCE_M = 0.20          # "arrived" radius for transit/step-across
POINT_PIVOT_RAD = math.radians(35.0)
STRIP_END_EPS_M = 0.05            # along-track slack at the far end
STRIP_END_REACH_M = 0.15          # ... or this close to the end point
STRIP_START_REACH_M = 0.30        # beyond this, walk over to the strip start
MAX_HEADING_ERROR_RAD = math.radians(50.0)

K_CROSS = 1.10                    # duty per metre of cross-track error
K_HEADING = 0.85                  # duty per radian of heading error

# A strip that is not moving for this long / this far is nudged, not killed.
STALL_WINDOW_S = 1.5
STALL_TRAVEL_M = 0.03
MAX_STALLS = 2
# Conservative floor used only to size per-strip deadlines.
STRIP_MIN_SPEED_MPS = 0.05
STRIP_BASE_TIME_S = 15.0
MAX_MISSION_S = 1800.0
TRANSIT_MAX_S = 300.0

# --------------------------------------------------------------- hardware ----
PROBE_DEPLOY_ANGLE = -20.0        # sensor_servo down into the soil
PROBE_RETRACT_ANGLE = -90.0
PROBE_SETTLE_S = 1.2
SOIL_CHECK_INTERVAL_M = 1.5

ULTRASONIC_PERIOD_S = 0.08        # ~12 Hz pings; the echo read is one behind
ULTRASONIC_MIN_VALID_CM = 2.0
OBSTACLE_THRESHOLD_CM = 22.0
OBSTACLE_CONFIRM_SAMPLES = 2
OBSTACLE_BACKOFF_M = ROVER_LENGTH_M * 0.40
OBSTACLE_DETOUR_M = ROVER_LENGTH_M * 0.9
MAX_EVASIONS_PER_STRIP = 4

# ------------------------------------------------------------------ state ----
auto_mode_active = False
auto_thread = None
auto_lock = threading.Lock()
_run_counter = 0
_current_run_id = 0
_pose_provider = None
_provider_error_ts = 0.0

auto_progress = {
    "phase": "idle",
    "strip": 0,
    "total_strips": 0,
    "coverage_pct": 0.0,
    "segments_done": 0,
    "obstacles": 0,
    "distance_m": 0.0,
    "incomplete_strips": [],
    "fault": "",
    "status": "idle",
    "message": "",
    "soil_state": "Not checked",
    "soil_checked_at": None,
    "soil_error": "",
    "watering_strip": False,
}


# ------------------------------------------------------------ configuration --
def set_pose_provider(provider):
    """Supply the fused pose, e.g. set_pose_provider(pose_tracker.snapshot).

    Anything callable returning {"x_m", "y_m", "heading_deg"} works.  Without
    it the controller dead-reckons tick deltas, which drifts but stays usable.
    """
    global _pose_provider
    if provider is not None and not callable(provider):
        raise TypeError("pose provider must be callable or None")
    _pose_provider = provider
    return True


def set_work_area(center_x, center_y, width_m, height_m, swath_m=None):
    """Define the sweep: centre point plus its two side lengths, in metres.

    Coordinates are in the same local frame the dashboard draws in (x east,
    y north, metres from the map origin), so the mission drives the rectangle
    exactly where the map shows it.
    """
    global AREA_CENTER_X, AREA_CENTER_Y, AREA_WIDTH_M, AREA_HEIGHT_M
    try:
        cx = float(center_x)
        cy = float(center_y)
        width = float(width_m)
        height = float(height_m)
    except (TypeError, ValueError):
        print("⚠️ Work area needs numeric centre and side lengths")
        return False
    if not all(math.isfinite(value) for value in (cx, cy, width, height)):
        print("⚠️ Work area values must be finite")
        return False
    if not (MIN_AREA_M <= width <= MAX_AREA_SIDE_M
            and MIN_AREA_M <= height <= MAX_AREA_SIDE_M):
        print(f"⚠️ Work area sides must be {MIN_AREA_M:.1f}-{MAX_AREA_SIDE_M:.0f} m")
        return False

    AREA_CENTER_X = cx
    AREA_CENTER_Y = cy
    AREA_WIDTH_M = width
    AREA_HEIGHT_M = height
    if swath_m is not None:
        set_swath(swath_m)
    summary = sweep_summary(width, height, SWATH_M)
    print(f"📐 Work area set: {width:.1f} m breadth x {height:.1f} m length "
          f"centred on ({cx:.1f}, {cy:.1f}) m · {summary['passes']} passes of "
          f"{height:.1f} m at {SWATH_M:.2f} m wide")
    return True


def clear_work_area():
    """Forget the area so a mission cannot start until a new one is set."""
    global AREA_CENTER_X, AREA_CENTER_Y, AREA_WIDTH_M, AREA_HEIGHT_M
    AREA_CENTER_X = AREA_CENTER_Y = None
    AREA_WIDTH_M = AREA_HEIGHT_M = 0.0
    print("📐 Work area cleared")
    return True


def has_work_area():
    return AREA_CENTER_X is not None and AREA_CENTER_Y is not None


def get_work_area():
    if not has_work_area():
        return None
    return {
        "center_x": AREA_CENTER_X,
        "center_y": AREA_CENTER_Y,
        "width_m": AREA_WIDTH_M,
        "height_m": AREA_HEIGHT_M,
        "swath_m": SWATH_M,
        "rover_width_m": ROVER_WIDTH_M,
        **sweep_summary(AREA_WIDTH_M, AREA_HEIGHT_M, SWATH_M),
    }


def get_coverage_config():
    return {
        "rover_width_m": ROVER_WIDTH_M,
        "swath_m": SWATH_M,
        "device_width_m": SWATH_M,
    }


def set_swath(swath_m):
    """Width of ground one pass covers, the denominator of the pass count."""
    global SWATH_M
    try:
        value = float(swath_m)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid device width {swath_m!r}")
        return False
    if not 0.10 <= value <= 5.0:
        print("⚠️ Device width must be between 0.10 m and 5.00 m")
        return False
    SWATH_M = value
    print(f"📏 Device width set to {SWATH_M:.2f} m")
    return True


def set_auto_speed(speed):
    """Cruise duty (0.2-1.0).  Kept across missions."""
    global AUTO_DRIVE_SPEED
    try:
        value = float(speed)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid auto speed {speed!r}")
        return False
    AUTO_DRIVE_SPEED = max(0.2, min(1.0, value))
    print(f"🚜 Auto speed set to {AUTO_DRIVE_SPEED:.2f} "
          f"(strip duty {_straight_speed():.2f})")
    return True


def set_rover_size(width_m, length_m=None):
    global ROVER_WIDTH_M, ROVER_LENGTH_M
    try:
        width = float(width_m)
        length = float(length_m) if length_m is not None else ROVER_LENGTH_M
    except (TypeError, ValueError):
        print("⚠️ Invalid rover size")
        return False
    ROVER_WIDTH_M = max(0.05, min(2.0, width))
    ROVER_LENGTH_M = max(0.05, min(2.0, length))
    return True


def is_auto_active():
    return bool(auto_mode_active)


def get_auto_progress():
    _update_tick_telemetry()
    return dict(auto_progress)


def enable_ultrasonic_in_auto():
    """Sensor use for auto mode; takes effect on the next mission start."""
    auto_progress["ultrasonic"] = True
    print("📡 Obstacle sensing enabled for auto mode")
    return True


def disable_ultrasonic_in_auto():
    auto_progress["ultrasonic"] = False
    print("📡 Obstacle sensing disabled for auto mode")
    return True


def toggle_ultrasonic_in_auto():
    enabled = not bool(auto_progress.get("ultrasonic", True))
    return enable_ultrasonic_in_auto() if enabled else disable_ultrasonic_in_auto()


# ------------------------------------------------------------- run object ----
class _Run:
    """Everything a single mission owns; nothing here outlives the thread."""

    def __init__(self, run_id, use_ultrasonic=True):
        self.id = run_id
        self.abort = threading.Event()
        self.started = time.monotonic()
        self.phase = "planning"
        self.strips = []
        self.index = 0
        self.total = 0
        self.covered_m = 0.0
        self.total_m = 0.0
        self.completed = 0
        self.incomplete = []
        self.obstacles = 0
        self.evasions_this_strip = 0
        self.fault = ""
        self.message = ""
        self.distance_since_soil = 0.0
        self.ultrasonic = use_ultrasonic
        self.sampler = _DistanceSampler()
        self.obstacle_hits = 0
        self.finished = False
        self.gps_heading_frozen = False
        self.fallback_pose = _EncoderPose(METRES_PER_TICK)

    def aborted(self):
        return bool(
            self.abort.is_set()
            or self.id != _current_run_id
            or not auto_mode_active
        )


class _DistanceSampler:
    """Ping on a schedule and read the echo that arrived since the last ping.

    get_distance_cm() triggers a ping and returns the previous echo, so
    hammering it in a 20 ms loop read the same value ~50 times a second.  Here
    a ping is issued at most every ULTRASONIC_PERIOD_S and the value is only
    marked fresh on the read that follows it.
    """

    def __init__(self, period=ULTRASONIC_PERIOD_S):
        self.period = period
        self._last_ping = 0.0
        self._value = None
        self._fresh = False

    def read(self, force=False):
        now = time.monotonic()
        if force or now - self._last_ping >= self.period:
            value = get_distance_cm()
            self._last_ping = now
            if value is None:
                self._fresh = False
            else:
                self._value = float(value)
                self._fresh = ULTRASONIC_MIN_VALID_CM <= self._value <= 400.0
        else:
            self._fresh = False
        return self._value, self._fresh


class _EncoderPose:
    """Fallback pose: dead-reckon tick deltas, never resetting the counters."""

    def __init__(self, metres_per_tick):
        self.metres_per_tick = max(1e-6, metres_per_tick)
        self._last = None
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def snapshot(self):
        imu = get_imu_snapshot()
        if imu.get("available"):
            self.heading = float(imu.get("heading_rad", 0.0))
        left, right = get_encoder_ticks()
        if self._last is not None:
            distance = (
                (left - self._last[0]) + (right - self._last[1])
            ) * 0.5 * self.metres_per_tick
            self.x += distance * math.cos(self.heading)
            self.y += distance * math.sin(self.heading)
        self._last = (left, right)
        return {
            "x_m": self.x,
            "y_m": self.y,
            "heading_deg": math.degrees(self.heading),
            "source": "encoders",
        }


# ------------------------------------------------------------------ utils ---
def _update_tick_telemetry():
    try:
        left, right = get_encoder_ticks()
        auto_progress["left_ticks"] = left
        auto_progress["right_ticks"] = right
    except Exception:
        pass
    return auto_progress


def _pose(run):
    global _provider_error_ts
    provider = _pose_provider or run.fallback_pose.snapshot
    try:
        snapshot = provider() or {}
    except Exception as exc:
        now = time.monotonic()
        if now - _provider_error_ts >= 5.0:
            _provider_error_ts = now
            print(f"⚠️ Pose provider failed ({exc}); using encoder pose")
        snapshot = run.fallback_pose.snapshot()
    try:
        return {
            "x_m": float(snapshot.get("x_m", 0.0)),
            "y_m": float(snapshot.get("y_m", 0.0)),
            "heading_deg": float(snapshot.get("heading_deg", 0.0)),
        }
    except (TypeError, ValueError):
        return {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}


def _heading_rad(run):
    imu = get_imu_snapshot()
    if imu.get("available"):
        return float(imu.get("heading_rad", 0.0))
    return math.radians(_pose(run)["heading_deg"])


def _pause(run, seconds):
    """Sleep in small slices so an abort is honoured within ~20 ms."""
    end = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < end:
        if run.aborted():
            return False
        time.sleep(min(LOOP_PERIOD_S, max(0.005, end - time.monotonic())))
    return not run.aborted()


def _halt():
    try:
        stop()
    except Exception:
        pass


def _halt_tools():
    _halt()
    try:
        WATER_PUMP.turnOffPump()
    except Exception:
        pass


def _retract_probe():
    try:
        sensor_servo.angle = PROBE_RETRACT_ANGLE
    except Exception:
        pass


def _freeze_gps_heading(run):
    """Stop GPS course from pulling the heading while strips are followed.

    Position fusion stays active; only the course blend fights the gyro-closed
    turns, and mid-strip it makes the rover weave.
    """
    owner = getattr(_pose_provider, "__self__", None)
    if owner is not None and hasattr(owner, "set_gps_heading_blend"):
        try:
            owner.set_gps_heading_blend(False)
            run.gps_heading_frozen = True
        except Exception as exc:
            print(f"⚠️ Could not freeze GPS heading ({exc})")


def _release_gps_heading(run):
    owner = getattr(_pose_provider, "__self__", None)
    if run.gps_heading_frozen and owner is not None:
        try:
            owner.set_gps_heading_blend(True)
        except Exception as exc:
            print(f"⚠️ Could not restore GPS heading ({exc})")
    run.gps_heading_frozen = False


def _status(run):
    if run.fault:
        return "fault"
    if run.finished:
        return "partial" if run.incomplete else "done"
    return "running"


def _publish(run, phase=None, message=None, **extra):
    if phase is not None:
        run.phase = phase
    if message is not None:
        run.message = message
    if not _is_current(run):
        return auto_progress
    total = run.total_m
    auto_progress.update({
        "phase": run.phase,
        "strip": (run.index + 1) if run.total else 0,
        "total_strips": run.total,
        "coverage_pct": round(100.0 * run.covered_m / total, 1) if total > 0 else 0.0,
        "segments_done": run.completed,
        "obstacles": run.obstacles,
        "distance_m": round(run.covered_m, 2),
        "incomplete_strips": list(run.incomplete),
        "fault": run.fault,
        "status": _status(run),
        "message": run.message,
    })
    auto_progress.update(extra)
    _update_tick_telemetry()
    return auto_progress


# ---------------------------------------------------------------- movement ---
def _drive_tank_safe(left, right):
    try:
        drive_tank(left, right)
    except Exception as exc:
        print(f"⚠️ Motor command failed ({exc})")


def _straight_speed():
    """Base duty for a strip: above the motors' floor, inside the usable band.

    Stopping the base duty at MIN_DUTY would leave the wheels on the edge of
    stalling with no downward steering authority left.
    """
    return max(MIN_DUTY + STEER_HEADROOM, min(1.0, AUTO_DRIVE_SPEED))


def _reverse_distance(run, metres):
    """Back up by an encoder-measured distance (relative delta, no reset)."""
    left0, right0 = get_encoder_ticks()
    target_ticks = max(1.0, abs(float(metres)) / METRES_PER_TICK)
    deadline = time.monotonic() + 4.0 + abs(float(metres)) * 5.0
    try:
        reverse(max(MIN_DUTY + STEER_HEADROOM, TURN_DUTY))
        while not run.aborted() and time.monotonic() < deadline:
            left, right = get_encoder_ticks()
            moved = (abs(left - left0) + abs(right - right0)) * 0.5
            if moved >= target_ticks:
                break
            time.sleep(LOOP_PERIOD_S)
    finally:
        _halt()
    _pause(run, 0.1)
    return not run.aborted()


def _pivot_to(run, target_rad):
    """Rotate in place to an absolute heading, closed on the gyro.

    The gyro is the authority; wheel-tracked arc length is only the fallback
    when the IMU is absent.  Either way the turn always ends with a timeout,
    so a lost sensor cannot leave the rover spinning.
    """
    target_rad = wrap_angle(target_rad)
    start_heading = _heading_rad(run)
    delta = abs(wrap_angle(target_rad - start_heading))
    if delta <= TURN_TOLERANCE_RAD:
        _halt()
        return True

    deadline = time.monotonic() + 5.0 + delta * 3.0
    imu_ready = bool(get_imu_snapshot().get("available"))
    left_ticks0, right_ticks0 = get_encoder_ticks()
    target_arc = max(0.0, delta - TURN_TOLERANCE_RAD) * (WHEEL_TRACK_M / 2.0)
    direction = 1.0 if wrap_angle(target_rad - start_heading) > 0 else -1.0

    while not run.aborted():
        if time.monotonic() > deadline:
            _halt()
            print(f"⚠️ Turn timed out {math.degrees(delta):.0f}° short; "
                  f"continuing on the current heading")
            return False

        if imu_ready:
            error = wrap_angle(target_rad - _heading_rad(run))
            if abs(error) <= TURN_TOLERANCE_RAD:
                _halt()
                _pause(run, TURN_SETTLE_S)
                return True
            if error > 0:
                turnLeft(TURN_DUTY)
            else:
                turnRight(TURN_DUTY)
        else:
            left, right = get_encoder_ticks()
            arc = (abs(left - left_ticks0) + abs(right - right_ticks0)) * 0.5 * METRES_PER_TICK
            if arc >= target_arc:
                _halt()
                _pause(run, TURN_SETTLE_S)
                return True
            if direction > 0:
                turnLeft(TURN_DUTY)
            else:
                turnRight(TURN_DUTY)

        time.sleep(LOOP_PERIOD_S)

    _halt()
    return False


def _obstacle_ahead(run):
    """Two consecutive fresh short readings count as an obstacle."""
    if not run.ultrasonic:
        return False
    distance, fresh = run.sampler.read()
    if not fresh or distance is None:
        return False
    if distance <= OBSTACLE_THRESHOLD_CM:
        run.obstacle_hits += 1
    else:
        run.obstacle_hits = 0
    return run.obstacle_hits >= OBSTACLE_CONFIRM_SAMPLES


def _drive_to_point(run, x, y, tolerance_m=POINT_TOLERANCE_M, timeout_s=None):
    """Drive to a point on the map, pivoting first when it is behind us."""
    pose = _pose(run)
    distance = math.hypot(x - pose["x_m"], y - pose["y_m"])
    if distance <= tolerance_m:
        return True
    deadline = time.monotonic() + (
        float(timeout_s) if timeout_s is not None else 10.0 + distance * 15.0
    )
    speed = _straight_speed()
    while not run.aborted():
        if time.monotonic() > deadline:
            _halt()
            print("⚠️ Could not reach the target point in time")
            return False

        pose = _pose(run)
        dx, dy = x - pose["x_m"], y - pose["y_m"]
        distance = math.hypot(dx, dy)
        if distance <= tolerance_m:
            _halt()
            return True

        target = math.atan2(dy, dx)
        error = wrap_angle(target - _heading_rad(run))
        if abs(error) > POINT_PIVOT_RAD:
            _halt()
            if not _pivot_to(run, target):
                return not run.aborted()
            continue

        correction = K_HEADING * error
        left = max(MIN_DUTY, min(1.0, speed - correction))
        right = max(MIN_DUTY, min(1.0, speed + correction))
        _publish(run, phase="transit",
                 message=f"driving to the area ({distance:.1f} m)")
        _drive_tank_safe(left, right)
        time.sleep(LOOP_PERIOD_S)

    _halt()
    return False


def _follow_segment(run, segment, *, max_distance=None, water=False,
                    check_soil=True):
    """Follow a straight segment with continuous cross-track correction.

    Returns (distance_driven_m, reason) with reason one of "done", "aborted",
    "obstacle", "soil", "stalled" or "timeout".
    """
    length = segment_length(segment)
    if length <= STRIP_END_EPS_M:
        return 0.0, "done"

    limit = length if max_distance is None else min(length, float(max_distance))
    pose = _pose(run)
    along, _cross = track_errors(pose["x_m"], pose["y_m"], segment)
    start_along = max(0.0, min(length, along))
    last_along = start_along
    driven = 0.0
    speed = _straight_speed()
    deadline = time.monotonic() + STRIP_BASE_TIME_S + limit / STRIP_MIN_SPEED_MPS
    stall_ref_time = time.monotonic()
    stall_ref_pose = (pose["x_m"], pose["y_m"])
    target_heading = segment_heading(segment)

    try:
        if water:
            if not WATER_PUMP.turnOnPump():
                raise RuntimeError("water pump did not turn on")

        while not run.aborted():
            pose = _pose(run)
            along, cross = track_errors(pose["x_m"], pose["y_m"], segment)
            along = max(0.0, min(length, along))
            driven = max(driven, along - start_along)
            end_gap = math.hypot(pose["x_m"] - segment["x1"],
                                 pose["y_m"] - segment["y1"])

            if along >= start_along + limit - STRIP_END_EPS_M:
                _halt()
                return driven, "done"
            if limit >= length - STRIP_END_EPS_M and end_gap <= STRIP_END_REACH_M:
                _halt()
                return driven, "done"

            if _obstacle_ahead(run):
                _halt()
                return driven, "obstacle"

            if check_soil and not water:
                run.distance_since_soil += max(0.0, along - last_along)
            last_along = along
            if (check_soil and not water
                    and run.distance_since_soil >= SOIL_CHECK_INTERVAL_M):
                _halt()
                return driven, "soil"

            # Mission-level time limit: never let one run occupy the rover
            # forever.
            now = time.monotonic()
            if now - run.started > MAX_MISSION_S:
                _halt()
                return driven, "timeout"

            # Stall watchdog: commanded motion without progress means a wheel
            # is buried or the gearbox is jammed against something.
            if now - stall_ref_time >= STALL_WINDOW_S:
                moved = math.hypot(pose["x_m"] - stall_ref_pose[0],
                                   pose["y_m"] - stall_ref_pose[1])
                if moved < STALL_TRAVEL_M:
                    _halt()
                    return driven, "stalled"
                stall_ref_time = now
                stall_ref_pose = (pose["x_m"], pose["y_m"])

            heading_error = wrap_angle(target_heading - _heading_rad(run))
            if abs(heading_error) > MAX_HEADING_ERROR_RAD:
                # Sideways or reversed (after a detour): line up again.
                _halt()
                if not _pivot_to(run, target_heading):
                    return driven, "timeout"
                continue

            left, right, _along, _cross, _err = steer_duties(
                segment, pose, speed, K_CROSS, K_HEADING, MIN_DUTY
            )
            _publish(run, along_m=round(along, 2), cross_track_m=round(cross, 2))
            _drive_tank_safe(left, right)
            time.sleep(LOOP_PERIOD_S)
    finally:
        if water:
            try:
                WATER_PUMP.turnOffPump()
            except Exception:
                pass

    _halt()
    return driven, "aborted"


# -------------------------------------------------------------- soil/water ---
def _soil_step(run):
    """Stop, deploy the probe, and return a fresh wet/dry reading."""
    _publish(run, phase="soil", message="checking soil moisture")
    _halt()
    if not _pause(run, 0.3):
        return "unknown"
    run.distance_since_soil = 0.0
    state = ""
    error = ""
    try:
        WATER_PUMP.turnOffPump()
    except Exception as exc:
        error = f"could not stop pump before soil check: {exc}"
    try:
        sensor_servo.angle = PROBE_DEPLOY_ANGLE
        if not _pause(run, PROBE_SETTLE_S):
            return "unknown"
        state = str(MOISTURE_SENSOR.getSoilMoistureState()).strip().lower()
    except Exception as exc:
        error = error or str(exc)
        print(f"⚠️ Soil check failed ({error})")
    finally:
        _retract_probe()
    if not _pause(run, 0.6):
        return "unknown"
    if error or state not in {"wet", "dry"}:
        diagnostics = MOISTURE_SENSOR.getDiagnostics()
        error = error or diagnostics.get("error") or "sensor returned no valid wet/dry state"
        state = "unknown"
    checked_at = time.strftime("%H:%M:%S")
    auto_progress.update({
        "soil_state": state.capitalize(),
        "soil_checked_at": checked_at,
        "soil_error": error,
    })
    _publish(run, phase="soil", message=f"soil {state} · {checked_at}",
             soil_state=state.capitalize(), soil_checked_at=checked_at,
             soil_error=error)
    print(f"🌱 Soil reads {state} at strip {run.index + 1} ({checked_at})")
    return state


# --------------------------------------------------------------- obstacles ---
def _sidestep(run, segment):
    """Back off, drive around the obstruction, then rejoin the same strip.

    The detour never has to be geometrically exact: the follower's cross-track
    term pulls the rover back onto the line afterwards.
    """
    heading = segment_heading(segment)
    side = 1.0 if run.obstacles % 2 else -1.0   # alternate sides
    print(f"🚧 Obstacle {run.obstacles} on strip {run.index + 1}: "
          f"backing off and stepping around it")
    _halt()
    if not _pause(run, 0.2):
        return False
    _reverse_distance(run, OBSTACLE_BACKOFF_M)

    detour_heading = wrap_angle(heading + side * math.pi / 2.0)
    if not _pivot_to(run, detour_heading):
        return not run.aborted()

    pose = _pose(run)
    target_x = pose["x_m"] + math.cos(detour_heading) * OBSTACLE_DETOUR_M
    target_y = pose["y_m"] + math.sin(detour_heading) * OBSTACLE_DETOUR_M
    _drive_to_point(run, target_x, target_y, timeout_s=30.0)
    _pivot_to(run, heading)
    return not run.aborted()


def _recover_stall(run, segment):
    """Nudge out of a stall: back off and re-align with the strip."""
    heading = segment_heading(segment)
    print(f"⚠️ No progress on strip {run.index + 1}; nudging free")
    _halt()
    if not _pause(run, 0.3):
        return False
    _reverse_distance(run, 0.2)
    _pivot_to(run, wrap_angle(heading + 0.3))
    _pivot_to(run, heading)
    return not run.aborted()


# ----------------------------------------------------------------- mission ---
def _orient_plan(pose, plan):
    """Work the plan from whichever end is nearest to the rover."""
    if len(plan) < 2:
        return plan
    first = plan[0]
    last = plan[-1]
    to_first = math.hypot(first["x0"] - pose["x_m"], first["y0"] - pose["y_m"])
    to_last = math.hypot(last["x1"] - pose["x_m"], last["y1"] - pose["y_m"])
    if to_last < to_first:
        flipped = []
        for segment in reversed(plan):
            flipped.append({
                "index": segment.get("index", 0),
                "x0": segment["x1"], "y0": segment["y1"],
                "x1": segment["x0"], "y1": segment["y0"],
                "length_m": segment_length(segment),
            })
        return flipped
    return plan


def _build_plan(run):
    """Strips covering the dashboard work area, in map coordinates."""
    if not has_work_area():
        raise RuntimeError("set the work area on the map before starting")
    plan = plan_rectangle_boustrophedon(
        AREA_CENTER_X, AREA_CENTER_Y, AREA_WIDTH_M, AREA_HEIGHT_M, SWATH_M,
    )
    if not plan:
        raise RuntimeError(
            f"a {AREA_WIDTH_M:.1f} x {AREA_HEIGHT_M:.1f} m area produced no strips"
        )
    summary = sweep_summary(AREA_WIDTH_M, AREA_HEIGHT_M, SWATH_M)
    start = plan[0]
    print(f"🗺️  Plan: {summary['passes']} passes of {summary['length_m']:.1f} m "
          f"across {summary['breadth_m']:.1f} m at {summary['device_width_m']:.2f} m "
          f"per pass (spacing {summary['lane_spacing_m']:.2f} m), "
          f"{total_planned_length(plan):.1f} m of passes; first pass starts at "
          f"({start['x0']:.1f}, {start['y0']:.1f}) heading "
          f"{math.degrees(segment_heading(start)):.0f}°")
    return plan


def _run_strip(run, segment):
    """Drive one strip, recovering from soil checks, stalls and obstacles.

    Returns True when the strip was covered.  Anything unrecoverable marks the
    strip incomplete and returns False; the mission then moves on.
    """
    stalls = 0
    run.evasions_this_strip = 0
    initial_soil = _soil_step(run)
    if initial_soil == "unknown":
        raise RuntimeError("soil moisture sensor is unavailable; autonomous watering stopped")
    water_strip = initial_soil == "dry"
    _publish(run,
             phase="watering" if water_strip else "driving",
             message=("dry soil · watering this strip" if water_strip
                      else "soil wet · checking at intervals"),
             watering_strip=water_strip)
    while not run.aborted():
        driven, reason = _follow_segment(run, segment, water=water_strip)
        if reason == "done":
            run.completed += 1
            run.covered_m += segment_length(segment)
            _publish(run, phase="driving", watering_strip=False,
                     message=f"strip {run.index + 1} complete")
            return True
        if reason == "aborted":
            return False
        if reason == "soil":
            state = _soil_step(run)
            if state == "unknown":
                raise RuntimeError("soil moisture sensor is unavailable; autonomous watering stopped")
            water_strip = state == "dry"
            _publish(run,
                     phase="watering" if water_strip else "driving",
                     message=("dry soil · watering rest of this strip" if water_strip
                              else "soil wet · resuming strip"),
                     watering_strip=water_strip)
            continue
        if reason == "obstacle":
            run.evasions_this_strip += 1
            run.obstacles += 1
            run.obstacle_hits = 0
            if run.evasions_this_strip > MAX_EVASIONS_PER_STRIP:
                print(f"⚠️ Strip {run.index + 1} stays blocked; leaving it incomplete")
                run.covered_m += driven
                return False
            _publish(run, phase="obstacle",
                     message=f"obstacle on strip {run.index + 1}")
            if not _sidestep(run, segment):
                return False
            continue
        if reason == "stalled":
            stalls += 1
            if stalls > MAX_STALLS:
                print(f"⚠️ Strip {run.index + 1} will not move; "
                      f"leaving it incomplete")
                run.covered_m += driven
                return False
            if not _recover_stall(run, segment):
                return False
            continue
        if reason == "timeout":
            print(f"⚠️ Strip {run.index + 1} did not finish in time; "
                  f"leaving it incomplete")
            run.covered_m += driven
            return False
    return False


def _follow_connector(run, connector):
    """Drive the short lane-change segment that makes the sweep continuous."""
    stalls = 0
    evasions = 0
    heading = segment_heading(connector)
    while not run.aborted():
        _publish(run, phase="turning", message="turning onto next strip")
        if not _pivot_to(run, heading):
            return False

        _publish(run, phase="transit", message="following strip-end connector")
        _driven, reason = _follow_segment(run, connector, check_soil=False)
        if reason == "done":
            return True
        if reason == "obstacle":
            evasions += 1
            run.obstacles += 1
            run.obstacle_hits = 0
            if evasions > MAX_EVASIONS_PER_STRIP or not _sidestep(run, connector):
                return False
            continue
        if reason == "stalled":
            stalls += 1
            if stalls > MAX_STALLS or not _recover_stall(run, connector):
                return False
            continue
        return False
    return False


def _mission(run):
    # Auto owns the drive hardware: discard any manual drive/pump state before
    # navigating to the first strip or taking a soil reading.
    _halt_tools()
    _retract_probe()
    _publish(run, phase="planning", message="planning the sweep")
    plan = _build_plan(run)
    _freeze_gps_heading(run)
    try:
        pose = _pose(run)
        plan = _orient_plan(pose, plan)
        run.strips = plan
        run.total = len(plan)
        run.total_m = total_planned_length(plan)
        _publish(run)

        first = plan[0]
        pose = _pose(run)
        gap = math.hypot(first["x0"] - pose["x_m"], first["y0"] - pose["y_m"])
        if gap > POINT_TOLERANCE_M:
            print(f"🚗 Driving {gap:.1f} m to the work area")
            _publish(run, phase="transit",
                     message=f"driving {gap:.1f} m to the work area")
            if not _drive_to_point(run, first["x0"], first["y0"],
                                   timeout_s=TRANSIT_MAX_S):
                if not run.aborted():
                    run.fault = "could not reach the work area"
                return

        align_next_strip = False
        for index, segment in enumerate(plan):
            if run.aborted():
                return
            if time.monotonic() - run.started > MAX_MISSION_S:
                run.fault = f"mission time limit ({MAX_MISSION_S:.0f} s) reached"
                return

            run.index = index
            heading = segment_heading(segment)

            # A successful previous pass is connected to this one by a
            # measured cross-lane segment below. Keep this point-drive only as
            # recovery when the previous strip or connector was incomplete.
            pose = _pose(run)
            start_gap = math.hypot(segment["x0"] - pose["x_m"],
                                   segment["y0"] - pose["y_m"])
            if align_next_strip or start_gap > STRIP_START_REACH_M:
                _publish(run, phase="turning",
                         message=f"moving onto strip {index + 1}/{run.total}")
                if not _drive_to_point(run, segment["x0"], segment["y0"],
                                       tolerance_m=0.12):
                    if run.aborted():
                        return
                    run.fault = f"could not reach strip {index + 1} start"
                    return
            align_next_strip = False
            if run.aborted():
                return
            _publish(run, phase="turning",
                     message=f"lining up strip {index + 1}/{run.total}")
            _pivot_to(run, heading)
            if run.aborted():
                return

            _publish(run, phase="driving",
                     message=f"strip {index + 1}/{run.total}")
            strip_complete = _run_strip(run, segment)
            if not strip_complete:
                if run.aborted():
                    return
                if index + 1 not in run.incomplete:
                    run.incomplete.append(index + 1)
                align_next_strip = True
            elif index + 1 < len(plan):
                connector = plan_connector(segment, plan[index + 1])
                if connector and not _follow_connector(run, connector):
                    if run.aborted():
                        return
                    align_next_strip = True
                    _publish(run, phase="transit",
                             message="connector missed · aligning with next strip")

        run.finished = True
        _publish(run, phase="done", message="sweep finished")
        print(f"✅ Coverage finished: {run.completed}/{run.total} strips, "
              f"{run.covered_m:.1f} m driven"
              + (f", incomplete {run.incomplete}" if run.incomplete else ""))
    finally:
        _release_gps_heading(run)
        _halt_tools()


def _is_current(run):
    return run.id == _current_run_id


def _finish(run, fault=""):
    global auto_mode_active
    if fault:
        run.fault = fault
        print(f"❌ Auto mission stopped: {fault}")
    # A mission that has already been replaced must not touch the hardware or
    # the progress the new mission is publishing.
    if not _is_current(run):
        print(f"ℹ️ Mission run {run.id} cleaned up after being superseded")
        return
    _halt_tools()
    _retract_probe()
    auto_mode_active = False
    # A finished sweep keeps its "done" phase so the dashboard can say so;
    # only a mission that never got going falls back to idle.
    _publish(run, phase="fault" if fault else ("done" if run.finished else "idle"))


def _mission_thread(run):
    try:
        _mission(run)
    except Exception as exc:
        _finish(run, f"{type(exc).__name__}: {exc}")
        return
    finally:
        if _is_current(run):
            _halt_tools()
            _retract_probe()
    _finish(run)


# -------------------------------------------------------------- public API ---
def start_auto_mode():
    """Start a mission; returns True when this rover is now running one."""
    global auto_mode_active, auto_thread, _run_counter, _current_run_id

    with auto_lock:
        if auto_mode_active and auto_thread is not None and auto_thread.is_alive():
            print("ℹ️ Auto mode already running")
            return True

        if not has_work_area():
            print("⚠️ No work area set - pick the area coordinates first")
            return False

        if auto_thread is not None and auto_thread.is_alive():
            auto_thread.join(timeout=2.5)
            if auto_thread.is_alive():
                print("⚠️ Previous mission is still stopping; try again shortly")
                return False

        _run_counter += 1
        _current_run_id = _run_counter
        auto_progress.update({
            "soil_state": "Not checked",
            "soil_checked_at": None,
            "soil_error": "",
            "watering_strip": False,
        })
        run = _Run(_current_run_id,
                   use_ultrasonic=bool(auto_progress.get("ultrasonic", True)))
        auto_mode_active = True
        auto_thread = threading.Thread(
            target=_mission_thread, args=(run,), name="auto-mission", daemon=True
        )
        auto_thread.start()

    print(f"🚜 Auto mode started (run {_current_run_id})")
    return True


def stop_auto_mode():
    """Stop the mission.  The worker notices within ~20 ms and releases the motors."""
    global auto_mode_active, _current_run_id

    with auto_lock:
        auto_mode_active = False
        _current_run_id = 0          # any other run id is stale from here on
        _halt_tools()
        _retract_probe()

    auto_progress["phase"] = "idle"
    auto_progress["status"] = "stopped"
    auto_progress["message"] = ""
    _update_tick_telemetry()
    print("⏹️ Auto mode stopped")
    return False


def toggle_auto_mode():
    return stop_auto_mode() if is_auto_active() else start_auto_mode()


__all__ = [
    "WHEEL_DIAMETER_M",
    "WHEEL_TRACK_M",
    "TICKS_PER_WHEEL_REV",
    "ROVER_WIDTH_M",
    "ROVER_LENGTH_M",
    "SWATH_M",
    "auto_progress",
    "set_pose_provider",
    "set_swath",
    "set_auto_speed",
    "set_work_area",
    "clear_work_area",
    "has_work_area",
    "get_work_area",
    "get_coverage_config",
    "set_rover_size",
    "start_auto_mode",
    "stop_auto_mode",
    "toggle_auto_mode",
    "is_auto_active",
    "get_auto_progress",
    "enable_ultrasonic_in_auto",
    "disable_ultrasonic_in_auto",
    "toggle_ultrasonic_in_auto",
]
