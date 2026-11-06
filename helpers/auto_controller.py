"""Autonomous coverage for the sprinkler rover.

The old controller drove the whole mission open-loop in wheel ticks, resetting
the encoders at every phase.  PoseTracker (main.py) consumes *absolute* tick
counts, so each reset teleported the reported pose backwards by more than a
metre, which then made GPS fusion reject good fixes as spikes.  Turns were
timed in ticks, the ultrasonic was polled ~50 times a second against an
asynchronous value, and an aborted mission could leave a second thread driving.

This version splits the job into three parts:

1. Planning  — helpers/auto_plan.py turns the area into ordered strips and
   provides the guidance maths.  Pure, testable, no hardware.
2. Following — one controller drives *any* straight segment using the fused
   pose (x, y, heading): heading is closed on the gyro and cross-track error is
   corrected continuously, so wheel slip, obstacle detours and uneven ground do
   not accumulate into missed strips.
3. Supervision — a small state machine owns the mission, the pump, the soil
   probe and every failure path.  A run id makes sure only the newest mission
   may command the motors.

Public API is unchanged apart from the additions set_pose_provider() and
set_swath(); main.py only needs one extra call.
"""

import math
import threading
import time

from helpers.auto_plan import (
    DEFAULT_OVERLAP,
    DEFAULT_SWATH_M,
    MIN_DUTY,
    plan_connector,
    plan_polygon,
    plan_strips,
    reverse_plan,
    rectangle_polygon,
    segment_heading,
    segment_length,
    steer_duties,
    strip_segment,
    to_world,
    total_planned_length,
    track_errors,
    trim_segment,
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

AREA_WIDTH_M = 10.0
AREA_HEIGHT_M = 10.0
AREA_POLYGON = []
# True when AREA_POLYGON holds map coordinates (drawn on the dashboard map) and
# must be driven where it is; False for the legacy rover-relative rectangle.
AREA_IS_MAPPED = False

# The wetted band of one pass, not the chassis width: overridable per install.
SWATH_M = DEFAULT_SWATH_M
SWATH_OVERLAP = DEFAULT_OVERLAP

# ---------------------------------------------------------------- driving ----
AUTO_DRIVE_SPEED = 0.70           # base duty while following a strip
TURN_DUTY = 0.55
TURN_TOLERANCE_RAD = math.radians(2.5)
TURN_SETTLE_S = 0.15
LOOP_PERIOD_S = 0.02

K_CROSS = 1.10                    # duty per metre of cross-track error
K_HEADING = 0.85                  # duty per radian of heading error

STALL_WINDOW_S = 1.2
STALL_TRAVEL_M = 0.03
MAX_STALLS = 3

# --------------------------------------------------------------- hardware ----
PROBE_DEPLOY_ANGLE = -20.0        # sensor_servo down into the soil
PROBE_RETRACT_ANGLE = -90.0
PROBE_SETTLE_S = 1.2
SOIL_CHECK_INTERVAL_M = 1.5
WATERING_RUN_M = 1.0

ULTRASONIC_PERIOD_S = 0.08        # ~12 Hz pings; the echo read is one behind
ULTRASONIC_MIN_VALID_CM = 2.0
OBSTACLE_THRESHOLD_CM = 22.0
OBSTACLE_CONFIRM_SAMPLES = 2
OBSTACLE_BACKOFF_M = ROVER_LENGTH_M * 0.45
OBSTACLE_DETOUR_M = ROVER_LENGTH_M * 0.9
MAX_EVASIONS_PER_STRIP = 6

MAX_MISSION_S = 1800.0
# How close to the first strip the rover must already be to skip the drive over.
TRANSIT_REACH_M = 0.60

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


def set_swath(swath_m, overlap=None):
    """Set the width of ground one pass wets (and optionally the overlap)."""
    global SWATH_M, SWATH_OVERLAP
    try:
        value = float(swath_m)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid swath {swath_m!r}")
        return False
    if not 0.10 <= value <= 5.0:
        print("⚠️ Swath must be between 0.10 m and 5.00 m")
        return False
    SWATH_M = value
    if overlap is not None:
        try:
            SWATH_OVERLAP = max(0.0, min(0.8, float(overlap)))
        except (TypeError, ValueError):
            pass
    print(f"📏 Swath set to {SWATH_M:.2f} m (overlap {SWATH_OVERLAP:.0%})")
    return True


def set_auto_speed(speed):
    global AUTO_DRIVE_SPEED
    try:
        value = float(speed)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid auto speed {speed!r}")
        return False
    AUTO_DRIVE_SPEED = max(0.2, min(1.0, value))
    print(f"🚜 Auto speed set to {AUTO_DRIVE_SPEED:.2f}")
    return True


def set_area(width_m, height_m=None):
    """Legacy rectangle, measured from the rover's own start pose and heading."""
    global AREA_WIDTH_M, AREA_HEIGHT_M, AREA_POLYGON, AREA_IS_MAPPED
    try:
        width = float(width_m)
        height = float(height_m) if height_m is not None else AREA_HEIGHT_M
    except (TypeError, ValueError):
        print("⚠️ Invalid area")
        return False
    if width < 0.5 or height < 0.5:
        print("⚠️ Area must be at least 0.5 m by 0.5 m")
        return False
    AREA_WIDTH_M = width
    AREA_HEIGHT_M = height
    AREA_POLYGON = rectangle_polygon(width, height)
    AREA_IS_MAPPED = False
    print(f"📐 Area set to {width:.1f} x {height:.1f} m from the rover's start")
    return True


def set_area_polygon(points):
    """Work area drawn on the map, in map metres (the dashboard's frame).

    The coordinates are kept exactly as given: the mission drives that polygon
    where it was drawn, not a copy anchored to the rover.
    """
    global AREA_POLYGON, AREA_WIDTH_M, AREA_HEIGHT_M, AREA_IS_MAPPED
    try:
        polygon = [[float(point[0]), float(point[1])] for point in points]
    except (TypeError, ValueError, IndexError):
        print("⚠️ Invalid area polygon")
        return False
    if len(polygon) < 3:
        print("⚠️ Area polygon needs at least three points")
        return False
    AREA_POLYGON = polygon
    AREA_WIDTH_M = max(point[0] for point in polygon) - min(point[0] for point in polygon)
    AREA_HEIGHT_M = max(point[1] for point in polygon) - min(point[1] for point in polygon)
    AREA_IS_MAPPED = True
    centre_x = sum(point[0] for point in polygon) / len(polygon)
    centre_y = sum(point[1] for point in polygon) / len(polygon)
    print(f"📐 Area polygon set: {len(polygon)} points, "
          f"{AREA_WIDTH_M:.1f} x {AREA_HEIGHT_M:.1f} m, "
          f"centre ({centre_x:.1f}, {centre_y:.1f}) m")
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
        self.stalls = 0
        self.fault = ""
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


def _sleep(run, seconds):
    """Sleep in small slices so an abort is honoured within ~20 ms."""
    end = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < end:
        if run.aborted():
            return False
        time.sleep(min(LOOP_PERIOD_S, max(0.005, end - time.monotonic())))
    return not run.aborted()


def _halt_tools():
    try:
        stop()
    except Exception:
        pass
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
    if run.total and run.completed >= run.total:
        return "done"
    if run.finished:
        return "partial"
    return "running"


def _publish(run, phase=None, **extra):
    if phase is not None:
        run.phase = phase
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
    })
    auto_progress.update(extra)
    _update_tick_telemetry()
    return auto_progress


# --------------------------------------------------------------- obstacles ---
def _obstacle_ahead(run):
    """Two consecutive fresh short readings count as an obstacle."""
    if not run.ultrasonic:
        return False
    distance, fresh = run.sampler.read()
    if not fresh:
        return False
    if distance <= OBSTACLE_THRESHOLD_CM:
        run.obstacle_hits += 1
    else:
        run.obstacle_hits = 0
    return run.obstacle_hits >= OBSTACLE_CONFIRM_SAMPLES


def _lane_clear(run):
    """Fresh confirmation that a sideways detour is not itself blocked."""
    if not run.ultrasonic:
        return True
    readings = []
    for _ in range(2):
        distance, fresh = run.sampler.read(force=True)
        readings.append(None if not fresh else distance)
        if not _sleep(run, 0.12):
            return False
    return all(value is None or value > OBSTACLE_THRESHOLD_CM for value in readings)


# ---------------------------------------------------------------- movement ---
def _drive_tank_safe(left, right):
    try:
        drive_tank(left, right)
    except Exception as exc:
        print(f"⚠️ Motor command failed ({exc})")


def _straight_speed():
    return max(MIN_DUTY, min(1.0, AUTO_DRIVE_SPEED))


def _follow_segment(run, segment, allow_soil_check=True):
    """Follow one straight segment with continuous cross-track correction.

    Returns (distance_driven_m, reason) with reason one of "done",
    "aborted", "obstacle", "soil", "stalled" or "timeout".
    """
    length = segment_length(segment)
    if length <= 0.02:
        return 0.0, "done"

    pose = _pose(run)
    along, _cross = track_errors(pose["x_m"], pose["y_m"], segment)
    start_along = max(0.0, min(length, along))
    last_along = start_along
    driven = 0.0
    stall_ref_time = time.monotonic()
    stall_ref_pose = (pose["x_m"], pose["y_m"])
    speed = _straight_speed()

    while not run.aborted():
        pose = _pose(run)
        along, cross = track_errors(pose["x_m"], pose["y_m"], segment)
        along = max(0.0, min(length, along))
        driven = max(driven, along - start_along)

        if along >= length - 0.02:
            stop()
            return driven, "done"

        if _obstacle_ahead(run):
            stop()
            return driven, "obstacle"

        run.distance_since_soil += max(0.0, along - last_along)
        last_along = along
        if allow_soil_check and run.distance_since_soil >= SOIL_CHECK_INTERVAL_M:
            stop()
            return driven, "soil"

        # Mission-level time limit: never let one run occupy the rover forever.
        now = time.monotonic()
        if now - run.started > MAX_MISSION_S:
            stop()
            return driven, "timeout"

        # Stall watchdog: commanded motion without progress means a wheel is
        # buried or the gearbox is jammed against something.
        if now - stall_ref_time >= STALL_WINDOW_S:
            moved = math.hypot(pose["x_m"] - stall_ref_pose[0],
                               pose["y_m"] - stall_ref_pose[1])
            if moved < STALL_TRAVEL_M:
                stop()
                return driven, "stalled"
            stall_ref_time = now
            stall_ref_pose = (pose["x_m"], pose["y_m"])

        left, right, _along, _cross, _err = steer_duties(
            segment, pose, speed, K_CROSS, K_HEADING, MIN_DUTY
        )
        _publish(run, along_m=round(along, 2), cross_track_m=round(cross, 2))
        _drive_tank_safe(left, right)
        time.sleep(LOOP_PERIOD_S)

    stop()
    return driven, "aborted"


def _turn_to_heading(run, target_rad, timeout_s=None):
    """Rotate in place to an absolute heading, closed on the gyro.

    The gyro is the authority; wheel-tracked arc length is only the fallback
    when the IMU is absent.  Either way the turn always ends with a timeout, so
    a lost sensor cannot leave the rover spinning.
    """
    start_heading = _heading_rad(run)
    delta = abs(wrap_angle(target_rad - start_heading))
    if delta <= TURN_TOLERANCE_RAD:
        stop()
        return True

    budget = timeout_s if timeout_s is not None else 4.0 + delta * 2.0
    deadline = time.monotonic() + budget
    imu_ready = bool(get_imu_snapshot().get("available"))
    left_ticks0, right_ticks0 = get_encoder_ticks()
    target_arc = max(0.0, delta - TURN_TOLERANCE_RAD) * (WHEEL_TRACK_M / 2.0)

    while not run.aborted():
        if time.monotonic() > deadline:
            stop()
            print("⚠️ Turn timed out; continuing on the current heading")
            return False

        if imu_ready:
            error = wrap_angle(target_rad - _heading_rad(run))
            if abs(error) <= TURN_TOLERANCE_RAD:
                stop()
                time.sleep(TURN_SETTLE_S)
                return True
            duty = max(MIN_DUTY * 0.8, min(TURN_DUTY, abs(error) * 1.6))
            if error > 0:
                turnLeft(duty)
            else:
                turnRight(duty)
        else:
            left, right = get_encoder_ticks()
            arc = (abs(left - left_ticks0) + abs(right - right_ticks0)) * 0.5 * METRES_PER_TICK
            if arc >= target_arc:
                stop()
                time.sleep(TURN_SETTLE_S)
                return True
            if wrap_angle(target_rad - start_heading) > 0:
                turnLeft(TURN_DUTY)
            else:
                turnRight(TURN_DUTY)

        time.sleep(LOOP_PERIOD_S)

    stop()
    return False


def _reverse_distance(run, metres, duty=0.45):
    """Back up by an encoder-measured distance (relative delta, no reset)."""
    left0, right0 = get_encoder_ticks()
    target_ticks = max(1.0, abs(metres) / METRES_PER_TICK)
    deadline = time.monotonic() + 2.0 + abs(metres) * 4.0
    reverse(max(MIN_DUTY, min(0.6, duty)))
    while not run.aborted() and time.monotonic() < deadline:
        left, right = get_encoder_ticks()
        moved = (abs(left - left0) + abs(right - right0)) * 0.5
        if moved >= target_ticks:
            break
        time.sleep(LOOP_PERIOD_S)
    stop()
    time.sleep(0.1)
    return not run.aborted()


def _avoid_obstacle(run, segment):
    """Back off, sidestep around the obstacle, then rejoin the same line.

    The detour never has to be geometrically exact: the follower's cross-track
    term pulls the rover back onto the strip afterwards.
    """
    run.evasions_this_strip += 1
    run.obstacles += 1
    run.obstacle_hits = 0
    _publish(run, phase="obstacle", evasions_this_strip=run.evasions_this_strip)
    print(f"🚧 Obstacle {run.obstacles} on strip {run.index + 1}: "
          f"backing off and sidestepping")

    _halt_tools()
    if not _sleep(run, 0.2):
        return False
    if not _reverse_distance(run, OBSTACLE_BACKOFF_M):
        return False

    heading = segment_heading(segment)
    side = 1.0 if run.evasions_this_strip % 2 else -1.0   # alternate sides
    for attempt in (side, -side):
        detour_heading = wrap_angle(heading + attempt * math.pi / 2.0)
        if not _turn_to_heading(run, detour_heading):
            return False
        if _lane_clear(run):
            break
    else:
        print("⚠️ Both sides look blocked; abandoning this strip")
        return False

    pose = _pose(run)
    start_x, start_y = pose["x_m"], pose["y_m"]
    end_x = start_x + math.cos(detour_heading) * OBSTACLE_DETOUR_M
    end_y = start_y + math.sin(detour_heading) * OBSTACLE_DETOUR_M
    detour = {
        "index": segment.get("index", 0),
        "x0": start_x, "y0": start_y,
        "x1": end_x, "y1": end_y,
        "length_m": OBSTACLE_DETOUR_M,
    }
    _follow_segment(run, detour, allow_soil_check=False)
    if not _turn_to_heading(run, heading):
        return False

    _publish(run, phase="driving")
    return not run.aborted()


def _stalled(run, segment):
    """Nudge out of a stall; three in a row is a fault worth stopping for."""
    run.stalls += 1
    print(f"⚠️ No progress on strip {run.index + 1} (stall {run.stalls}/{MAX_STALLS})")
    _halt_tools()
    if run.stalls >= MAX_STALLS:
        return False
    if not _sleep(run, 0.3):
        return False
    _reverse_distance(run, 0.15)
    _turn_to_heading(run, segment_heading(segment) + (0.25 if run.stalls % 2 else -0.25))
    _turn_to_heading(run, segment_heading(segment))
    return not run.aborted()


# -------------------------------------------------------------- soil/water ---
def _water_along(run, segment, metres=WATERING_RUN_M):
    """Drive a stretch with the pump on, cross-track corrected like a strip."""
    pose = _pose(run)
    along, _cross = track_errors(pose["x_m"], pose["y_m"], segment)
    patch = trim_segment(segment, max(0.0, along), metres)
    if patch["length_m"] <= 0.02:
        return 0.0
    _publish(run, phase="watering")
    try:
        WATER_PUMP.turnOnPump()
        driven, reason = _follow_segment(run, patch, allow_soil_check=False)
    finally:
        WATER_PUMP.turnOffPump()
        _halt_tools()
    print(f"💧 Watered {driven:.2f} m ({reason})")
    return driven


def _soil_step(run, segment):
    """Probe the soil, water the stretch ahead when it reads dry."""
    _publish(run, phase="checking_soil")
    _halt_tools()
    if not _sleep(run, 0.3):
        return
    run.distance_since_soil = 0.0
    try:
        sensor_servo.angle = PROBE_DEPLOY_ANGLE
        if not _sleep(run, PROBE_SETTLE_S):
            return
        state = str(MOISTURE_SENSOR.getSoilMoistureState()).strip().lower()
        _retract_probe()
        if not _sleep(run, 0.6):
            return
        print(f"🌱 Soil reads {state or 'unknown'} at strip {run.index + 1}")
        if state == "dry":
            _water_along(run, segment)
        else:
            _publish(run, phase="driving")
    except Exception as exc:
        print(f"⚠️ Soil check failed ({exc})")
    finally:
        _retract_probe()


# ----------------------------------------------------------------- mission ---
def _orient_plan(pose, plan):
    """Work the plan from whichever end is nearest to the rover."""
    if len(plan) < 2:
        return plan
    first = plan[0]
    last = plan[-1]
    to_first = math.hypot(first["x0"] - pose["x_m"], first["y0"] - pose["y_m"])
    to_last = math.hypot(last["x1"] - pose["x_m"], last["y1"] - pose["y_m"])
    return plan if to_first <= to_last else reverse_plan(plan)


def _build_plan(run):
    """Strips in map coordinates.

    A dashboard work area is a polygon on the map, so it is driven exactly
    where it was drawn.  The legacy rectangle (set_area, or no area at all) is
    measured from the rover's own start pose and heading instead.
    """
    pose = _pose(run)

    if AREA_POLYGON and AREA_IS_MAPPED:
        plan = plan_polygon(AREA_POLYGON, SWATH_M, SWATH_OVERLAP)
        source = "map polygon"
    else:
        polygon = AREA_POLYGON or rectangle_polygon(AREA_WIDTH_M, AREA_HEIGHT_M)
        origin_x, origin_y = pose["x_m"], pose["y_m"]
        heading = math.radians(pose["heading_deg"])
        plan = [
            to_world(strip_segment(strip, index), origin_x, origin_y, heading)
            for index, strip in enumerate(plan_strips(polygon, SWATH_M, SWATH_OVERLAP))
        ]
        source = "rover-relative rectangle"

    if not plan:
        raise RuntimeError("the work area produced no strips")

    plan = _orient_plan(pose, plan)
    run.strips = plan
    run.total = len(plan)
    run.total_m = total_planned_length(plan)
    start = plan[0]
    print(f"🗺️  Plan: {run.total} strips, {run.total_m:.1f} m, "
          f"swath {SWATH_M:.2f} m ({source}), first strip starts at "
          f"({start['x0']:.1f}, {start['y0']:.1f}) heading "
          f"{math.degrees(segment_heading(start)):.0f}°")
    return plan


def _run_segment(run, segment, check_soil=True, count_coverage=True):
    """Drive one segment to the end, recovering from soil checks and obstacles."""
    while not run.aborted():
        driven, reason = _follow_segment(run, segment, allow_soil_check=check_soil)
        if reason == "done":
            if count_coverage:
                run.completed += 1
                run.covered_m += segment_length(segment)
            run.stalls = 0
            _publish(run, phase="driving")
            return True
        if reason == "aborted":
            return False
        if reason == "soil":
            _soil_step(run, segment)
            continue
        if reason == "obstacle":
            _publish(run, phase="obstacle")
            if run.evasions_this_strip >= MAX_EVASIONS_PER_STRIP:
                if count_coverage:
                    print(f"⚠️ Strip {run.index + 1} blocked after "
                          f"{run.evasions_this_strip} evasions; leaving it incomplete")
                    run.incomplete.append(run.index + 1)
                    run.covered_m += driven
                    _publish(run, phase="driving")
                return False
            if not _avoid_obstacle(run, segment):
                if run.aborted():
                    return False
                if count_coverage:
                    run.incomplete.append(run.index + 1)
                    run.covered_m += driven
                return False
            continue
        if reason == "stalled":
            if not _stalled(run, segment):
                run.fault = f"stalled on strip {run.index + 1}"
                return False
            continue
        if reason == "timeout":
            run.fault = f"mission time limit ({MAX_MISSION_S:.0f} s) reached"
            return False
    return False


def _run_strip(run, segment):
    return _run_segment(run, segment)


def _transit_to(run, segment):
    """Drive over to the work area: no soil probing and no coverage credit."""
    _publish(run, phase="transit")
    if not _turn_to_heading(run, segment_heading(segment)):
        return False
    return _run_segment(run, segment, check_soil=False, count_coverage=False)


def _mission(run):
    _publish(run, phase="planning")
    _freeze_gps_heading(run)

    plan = _build_plan(run)
    run.stalls = 0

    first = plan[0]
    pose = _pose(run)
    start_gap = math.hypot(first["x0"] - pose["x_m"], first["y0"] - pose["y_m"])
    if start_gap > TRANSIT_REACH_M:
        print(f"🚗 Driving {start_gap:.1f} m to the first strip")
        transit = {
            "index": 0,
            "x0": pose["x_m"], "y0": pose["y_m"],
            "x1": first["x0"], "y1": first["y0"],
            "length_m": start_gap,
        }
        if not _transit_to(run, transit):
            if not run.aborted():
                run.fault = "could not reach the work area"
            return
        if not _turn_to_heading(run, segment_heading(first)):
            run.fault = "could not line up with the first strip"
            return
    _publish(run, phase="driving")

    for index, segment in enumerate(plan):
        if run.aborted():
            return
        run.index = index
        run.evasions_this_strip = 0
        run.stalls = 0
        _publish(run, phase="driving")

        completed = _run_strip(run, segment)
        if run.aborted():
            return
        if not completed and run.fault:
            return

        following = plan[index + 1] if index + 1 < len(plan) else None
        if following is None:
            break

        connector = plan_connector(segment, following)
        if connector is None:
            continue
        _publish(run, phase="turning")
        if not _turn_to_heading(run, segment_heading(connector)):
            run.fault = f"turn failed after strip {index + 1}"
            return
        _follow_segment(run, connector, allow_soil_check=False)
        if run.aborted():
            return
        if not _turn_to_heading(run, segment_heading(following)):
            run.fault = f"turn failed entering strip {index + 2}"
            return

    run.finished = True
    run.phase = "done"
    print(f"✅ Coverage finished: {run.completed}/{run.total} strips, "
          f"{run.covered_m:.1f} m driven"
          + (f", incomplete {run.incomplete}" if run.incomplete else ""))


def _is_current(run):
    return run.id == _current_run_id


def _finish(run, fault=""):
    global auto_mode_active
    if fault:
        run.fault = fault
        print(f"❌ Auto mission stopped: {fault}")
    _release_gps_heading(run)
    # A mission that has already been replaced must not touch the hardware or
    # the progress the new mission is publishing.
    if not _is_current(run):
        print(f"ℹ️ Mission run {run.id} cleaned up after being superseded")
        return
    _halt_tools()
    _retract_probe()
    auto_mode_active = False
    _publish(run, phase="fault" if fault else "idle")


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

        if auto_thread is not None and auto_thread.is_alive():
            auto_thread.join(timeout=2.5)
            if auto_thread.is_alive():
                print("⚠️ Previous mission is still stopping; try again shortly")
                return False

        _run_counter += 1
        _current_run_id = _run_counter
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
    "AREA_WIDTH_M",
    "AREA_HEIGHT_M",
    "AREA_POLYGON",
    "AUTO_DRIVE_SPEED",
    "SWATH_M",
    "SWATH_OVERLAP",
    "auto_progress",
    "set_pose_provider",
    "set_swath",
    "set_auto_speed",
    "set_area",
    "set_area_polygon",
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
