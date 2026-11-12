"""Bounded lawnmower sweep with moisture-triggered watering.

The rover does not need to be pointed north.  Pressing Start captures the
current MPU6050 heading (circular mean of several readings, rover stationary)
as the reference; everything afterwards is relative to it:

    relative 0 deg   along the heading the rover had at Start
    relative 180 deg back along it (the return row)
    width            extends to the side given by ``turn_side``

A mission is: DRIVE_ROW -> [TURN_1 -> STEP -> TURN_2 -> DRIVE_ROW] ... and is
finished after ``rows = ceil(width_m / row_spacing_m)`` rows.  Every
``water_interval_m`` of travel the rover stops, reads the soil probe and runs
the pump for ``pump_run_seconds`` when the soil is dry.

Safety rules followed throughout:

* the stop flag is checked in every loop (including pump run time),
* any exception, timeout or lost sensor ends with motors off and pump off,
* ``stop()`` is only issued on state changes, never inside the drive loop,
* all driving is closed on the gyro heading and on encoder distance.

The thread is a daemon and owns no Flask state, so the HTTP server never
blocks.  See ``start()`` / ``stop()`` / ``get_status()`` at the bottom.
"""

import math
import os
import threading
import time

from helpers import auto_plan
from helpers import encoder_controller as encoders
from helpers import motor_controls as motors
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.hmc5883l import reset_compass_fusion
from helpers.mpu6050 import calibrate_imu, get_imu_snapshot
from helpers.pump_controller import WATER_PUMP
from helpers.ultrasonic_controller import get_distance_cm, is_ultrasonic_enabled

# ------------------------------------------------------------------ states --
IDLE = "IDLE"
CALIBRATE = "CALIBRATE"
DRIVE_ROW = "DRIVE_ROW"
TURN_1 = "TURN_1"
STEP = "STEP"
TURN_2 = "TURN_2"
WATER = "WATER"
AVOID = "AVOID"
DONE = "DONE"
ERROR = "ERROR"

# ------------------------------------------------------------- parameters --
DEFAULT_PARAMS = {
    "length_m": 5.0,             # row length, along the Start heading
    "width_m": 3.0,              # area width, to `turn_side`
    "row_spacing_m": 0.5,        # sideways step between rows
    "turn_side": "right",        # which way the width extends
    "water_interval_m": 2.0,     # travel between soil checks (0 = never)
    "pump_run_seconds": 3.0,     # watering time when the probe reads dry
    "drive_speed": 0.85,         # cruise duty, 0..1
    "turn_speed": 0.90,          # pivot duty, 0..1
}

PARAM_LIMITS = {
    "length_m": (0.20, 100.0),
    "width_m": (0.10, 100.0),
    "row_spacing_m": (0.05, 10.0),
    "water_interval_m": (0.0, 100.0),
    "pump_run_seconds": (0.0, 120.0),
    "drive_speed": (0.10, 1.0),
    "turn_speed": (0.10, 1.0),
}

# ---------------------------------------------------------------- driving --
LOOP_PERIOD_S = 0.05            # 20 Hz row loop; light enough for the Zero W
TURN_LOOP_PERIOD_S = 0.02       # 50 Hz while pivoting (fast angle change)
HEADING_GAIN = 0.60             # duty per radian of heading error
MAX_STEER_CORRECTION = 0.12     # cap on the differential correction
HEADING_DEADBAND_RAD = math.radians(0.5)
MIN_DRIVE_DUTY = auto_plan.MIN_DRIVE_DUTY
TURN_TOLERANCE_DEG = 4.0
TURN_SETTLE_S = 0.25
TURN_TIMEOUT_PER_90_S = 8.0     # budget per 90 deg of pivot, scaled by angle
MIN_ROW_SPEED_MPS = 0.03        # used only to size distance timeouts
ROW_TIMEOUT_PAD_S = 20.0
STEP_TIMEOUT_PAD_S = 15.0
MAX_MISSION_S = 1800.0

# Metres of ground per encoder tick.  Confirm with calibrate_wheels.py before
# trusting row lengths; everything distance-based scales with this number.
METRES_PER_TICK = (
    math.pi * encoders.WHEEL_DIAMETER_M / max(1, encoders.TICKS_PER_WHEEL_REV)
)
TICKS_PER_METRE = 1.0 / METRES_PER_TICK

# -------------------------------------------------------------- reference --
REFERENCE_SETTLE_S = 0.40       # let the chassis stop rocking
REFERENCE_SAMPLES = 20
REFERENCE_SAMPLE_S = 0.03

# --------------------------------------------------------------- obstacle --
ULTRASONIC_THRESHOLD_CM = 25.0
ULTRASONIC_MIN_VALID_CM = 2.0   # 0.0 means "no echo yet", not "wall"
ULTRASONIC_CONFIRM_SAMPLES = 2
ULTRASONIC_PERIOD_S = 0.12
# _path_clear() requires this many blocked samples out of 3 before treating
# the path as still occupied.  Must be > 0 and <= 3.  Keeping it at 2
# (same threshold as the drive-loop confirmation) means a single stale reading
# cannot re-block a path that has just cleared.
OBSTACLE_CLEAR_THRESHOLD = 2
OBSTACLE_WAIT_S = 2.0
OBSTACLE_MAX_RETRIES = 3
OBSTACLE_SAMPLE_S = 0.15
SIDESTEP_M = 0.35

# ------------------------------------------------------------------- misc --
SOIL_RETRY_S = 0.20
STOP_JOIN_TIMEOUT_S = 3.0
STOP_CHECK_S = 0.02
# On a dashboard disconnect the failsafe stops the sweep by default (same
# policy as manual driving).  Set AUTO_STOP_ON_DISCONNECT=0 for unattended
# runs; an explicit stop still always wins.
AUTO_STOP_ON_DISCONNECT = os.environ.get(
    "AUTO_STOP_ON_DISCONNECT", "1"
).strip().lower() not in {"0", "false", "no", "off"}

_lock = threading.RLock()
_thread = None
_active_run = None

_status = {
    "active": False,
    "state": IDLE,
    "row": 0,
    "total_rows": 0,
    "progress_pct": 0.0,
    "heading_error_deg": 0.0,
    "row_distance_m": 0.0,
    "distance_m": 0.0,
    "last_moisture": "not checked",
    "last_moisture_at": None,
    "watered_count": 0,
    "obstacles": 0,
    "pump_active": False,
    "ultrasonic_active": False,
    "reference_heading_deg": None,
    "message": "",
    "fault": "",
    "started_at": None,
    "elapsed_s": 0.0,
    "params": dict(DEFAULT_PARAMS),
    "metres_per_tick": METRES_PER_TICK,
}


class _StopRequested(Exception):
    """Raised inside the mission thread when stop() is called."""


class _AutoFault(Exception):
    """A condition that must abort the mission and report ERROR."""


def _publish(**fields):
    with _lock:
        _status.update(fields)


def _normalise_params(raw):
    """Merge user parameters over the defaults and reject nonsense values."""
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("params must be an object")
    merged = dict(DEFAULT_PARAMS)
    merged.update({key: value for key, value in raw.items() if key in DEFAULT_PARAMS})
    unknown = [key for key in raw if key not in DEFAULT_PARAMS]
    if unknown:
        raise ValueError("unknown parameter(s): " + ", ".join(sorted(unknown)))

    clean = {}
    for name, (low, high) in PARAM_LIMITS.items():
        try:
            value = float(merged[name])
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a number")
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        if not low <= value <= high:
            raise ValueError(f"{name} must be between {low} and {high}")
        clean[name] = value

    side = str(merged["turn_side"]).strip().lower()
    if side not in ("right", "left"):
        raise ValueError("turn_side must be 'right' or 'left'")
    clean["turn_side"] = side

    return clean


class _Sweep:
    """One mission.  Owns its own stop flag; nothing outlives the thread."""

    def __init__(self, params):
        self.params = params
        self.length = params["length_m"]
        self.width = params["width_m"]
        self.spacing = params["row_spacing_m"]
        self.turn_sign = auto_plan.turn_sign(params["turn_side"])
        self.total_rows = auto_plan.row_count(self.width, self.spacing)

        self.stop_event = threading.Event()
        self.stop_reason = "stop requested"
        self.state = IDLE
        self.reference_rad = 0.0
        self.row = 0
        self.row_distance = 0.0
        self.row_since_check = 0.0
        self.distance_m = 0.0
        self.heading_error = 0.0
        self.last_moisture = "not checked"
        self.last_moisture_at = None
        self.watered_count = 0
        self.obstacles = 0
        self.sidestep_used = False
        self.started_monotonic = time.monotonic()
        self.started_at = time.time()
        self._last_publish = 0.0   # rate-limit _publish_progress() to ~5 Hz

    # ------------------------------------------------------------ helpers --
    def _heading(self):
        snapshot = get_imu_snapshot()
        if not snapshot.get("available"):
            raise _AutoFault("MPU6050 heading unavailable")
        return math.radians(float(snapshot.get("heading_deg", 0.0)))

    def _abs(self, relative_deg):
        return auto_plan.target_rad(self.reference_rad, relative_deg)

    def _set_state(self, state, message=None):
        self.state = state
        fields = {"state": state}
        if message is not None:
            fields["message"] = message
        _publish(**fields)

    def _check_mission_time(self):
        if time.monotonic() - self.started_monotonic > MAX_MISSION_S:
            self._halt()
            raise _AutoFault(f"mission time limit ({MAX_MISSION_S:.0f} s) reached")

    def _sleep(self, seconds):
        """Sleep in small slices so a stop request is never ignored."""
        deadline = time.monotonic() + max(0.0, float(seconds))
        while True:
            if self.stop_event.is_set():
                raise _StopRequested()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            self._check_mission_time()
            time.sleep(min(STOP_CHECK_S, remaining))

    def _halt(self):
        try:
            motors.stop()
        except Exception as exc:
            print(f"⚠️ Auto mode: motor stop failed ({exc})")
        try:
            if WATER_PUMP.isPumpOn():
                WATER_PUMP.turnOffPump()
        except Exception as exc:
            print(f"⚠️ Auto mode: pump stop failed ({exc})")
        _publish(pump_active=False)

    def _progress_pct(self):
        if self.total_rows <= 0:
            return 0.0
        fraction = (
            self.row + auto_plan.row_progress(self.row_distance, self.length)
        ) / self.total_rows
        return round(100.0 * max(0.0, min(1.0, fraction)), 1)

    def _publish_progress(self, force=False):
        now = time.monotonic()
        if not force and (now - self._last_publish < 0.18):
            return
        self._last_publish = now
        _publish(
            row=self.row + 1,
            total_rows=self.total_rows,
            progress_pct=self._progress_pct(),
            heading_error_deg=round(math.degrees(self.heading_error), 1),
            row_distance_m=round(self.row_distance, 2),
            distance_m=round(self.distance_m, 2),
            last_moisture=self.last_moisture,
            last_moisture_at=self.last_moisture_at,
            watered_count=self.watered_count,
            obstacles=self.obstacles,
            elapsed_s=round(time.monotonic() - self.started_monotonic, 1),
        )

    # -------------------------------------------------------------- steps --
    def _calibrate(self):
        """Capture the Start heading as the relative frame's zero."""
        self._set_state(CALIBRATE, "capturing heading reference")
        self._halt()
        self._sleep(REFERENCE_SETTLE_S)

        if not is_ultrasonic_enabled():
            print("⚠️ Auto mode: ultrasonic sensor disabled — obstacle detection off")

        try:
            calibrate_imu(samples=100)
        except Exception as exc:
            print(f"⚠️ Auto mode: IMU bias calibration failed ({exc})")

        samples = []
        for _ in range(REFERENCE_SAMPLES):
            if self.stop_event.is_set():
                raise _StopRequested()
            samples.append(self._heading())
            self._sleep(REFERENCE_SAMPLE_S)
        self.reference_rad = auto_plan.circular_mean_rad(samples)

        try:
            reset_compass_fusion()
        except Exception as exc:
            print(f"⚠️ Auto mode: compass fusion reset failed ({exc})")

        _publish(reference_heading_deg=round(math.degrees(self.reference_rad), 1))
        print(
            "🧭 Auto mode reference heading captured at "
            f"{math.degrees(self.reference_rad):.1f} deg; "
            f"{self.total_rows} rows of {self.length:.1f} m, "
            f"{self.spacing:.2f} m apart, extending {self.params['turn_side']}"
        )

    def _drive(self, distance_m, target_heading_rad, timeout_s, track_row=True):
        """Drive straight for ``distance_m`` metres, holding a heading.

        Returns ``"done"``, ``"water"`` (soil check is due) or ``"obstacle"``
        (something is in front); the caller owns the state change.  Distance
        is the mean absolute wheel travel, which tolerates swapped or inverted
        encoder channels.

        Note: get_distance_cm() fires the ultrasonic trigger and returns the
        previous echo's reading (the callback updates asynchronously).  The
        reading is therefore at most ULTRASONIC_PERIOD_S old — harmless at
        driving speeds, and the ULTRASONIC_MIN_VALID_CM guard filters the
        initial 0.0 value before the first echo arrives.
        """
        time.sleep(0.05)
        left_last, right_last = encoders.get_encoder_ticks()
        travelled = 0.0
        deadline = time.monotonic() + max(0.5, float(timeout_s))
        next_obstacle = 0.0
        obstacle_hits = 0

        while travelled < distance_m:
            if self.stop_event.is_set():
                raise _StopRequested()
            now = time.monotonic()
            if now > deadline:
                self._halt()
                raise _AutoFault(
                    f"distance timeout after {travelled:.2f} of {distance_m:.2f} m"
                )
            self._check_mission_time()

            left, right = encoders.get_encoder_ticks()
            step = (
                abs(left - left_last) + abs(right - right_last)
            ) * 0.5 * METRES_PER_TICK
            left_last, right_last = left, right
            travelled += step
            self.distance_m += step
            if track_row:
                self.row_distance += step
                self.row_since_check += step

            if (
                track_row
                and self.params["water_interval_m"] > 0.0
                and self.row_since_check >= self.params["water_interval_m"]
            ):
                self._publish_progress(force=True)
                return "water"

            if is_ultrasonic_enabled() and now >= next_obstacle:
                next_obstacle = now + ULTRASONIC_PERIOD_S
                distance_cm = get_distance_cm()
                if (
                    distance_cm is not None
                    and ULTRASONIC_MIN_VALID_CM <= distance_cm <= ULTRASONIC_THRESHOLD_CM
                ):
                    obstacle_hits += 1
                else:
                    obstacle_hits = 0
                if obstacle_hits >= ULTRASONIC_CONFIRM_SAMPLES:
                    self._publish_progress(force=True)
                    return "obstacle"

            self.heading_error = auto_plan.wrap_rad(
                target_heading_rad - self._heading()
            )
            if abs(self.heading_error) < HEADING_DEADBAND_RAD:
                self.heading_error = 0.0
            left_duty, right_duty = auto_plan.steer_duties(
                self.heading_error,
                self.params["drive_speed"],
                HEADING_GAIN,
                MAX_STEER_CORRECTION,
                MIN_DRIVE_DUTY,
            )
            motors.drive_tank(left_duty, right_duty)
            self._publish_progress()
            time.sleep(LOOP_PERIOD_S)

        self.heading_error = 0.0
        self._publish_progress(force=True)
        return "done"

    def _turn_to(self, target_heading_rad):
        """Pivot on the spot until the gyro heading reaches the target."""
        tolerance = math.radians(TURN_TOLERANCE_DEG)
        angle = abs(auto_plan.wrap_rad(target_heading_rad - self._heading()))
        timeout = TURN_TIMEOUT_PER_90_S * max(1.0, angle / (math.pi / 2.0))
        deadline = time.monotonic() + timeout

        while True:
            if self.stop_event.is_set():
                raise _StopRequested()
            self._check_mission_time()
            error = auto_plan.wrap_rad(target_heading_rad - self._heading())
            self.heading_error = error
            self._publish_progress()

            if abs(error) <= tolerance:
                self._halt()
                self._sleep(TURN_SETTLE_S)
                if abs(
                    auto_plan.wrap_rad(target_heading_rad - self._heading())
                ) <= tolerance * 2.0:
                    self.heading_error = 0.0
                    self._publish_progress(force=True)
                    return
                continue

            if time.monotonic() > deadline:
                self._halt()
                raise _AutoFault(
                    f"turn timeout {math.degrees(error):.0f} deg short of target"
                )

            # Positive error means the target is CCW (left) of the nose, so
            # pivot CCW: left wheel reverse, right wheel forward.
            # drive_tank(-speed, +speed) does that; direction flips it.
            direction = 1.0 if error > 0.0 else -1.0
            speed = self.params["turn_speed"]
            motors.drive_tank(-direction * speed, direction * speed)
            time.sleep(TURN_LOOP_PERIOD_S)

    def _soil_is_dry(self):
        """Check whether the soil is dry using the moisture controller's public API.

        Uses MOISTURE_SENSOR.getSoilMoistureState() so that the
        wet_when_pressed polarity env-var is respected consistently, rather
        than re-interpreting the raw GPIO level here.
        """
        for _ in range(2):
            try:
                state = MOISTURE_SENSOR.getSoilMoistureState()
            except Exception as exc:
                print(f"⚠️ Auto mode: soil read failed ({exc})")
                self._sleep(SOIL_RETRY_S)
                continue
            if state == "Unknown":
                print("⚠️ Auto mode: moisture sensor returned Unknown, retrying")
                self._sleep(SOIL_RETRY_S)
                continue
            return state == "Dry"
        raise _AutoFault("soil moisture sensor unreadable")

    def _water(self):
        """Stop, check the soil and water for the configured time if dry."""
        self._set_state(WATER, "checking soil moisture")
        self._halt()
        dry = self._soil_is_dry()
        self.last_moisture = "dry" if dry else "wet"
        self.last_moisture_at = time.time()
        if dry:
            self._set_state(WATER, f"watering for {self.params['pump_run_seconds']:.1f} s")
            WATER_PUMP.turnOnPump()
            _publish(pump_active=True)
            try:
                self._sleep(self.params["pump_run_seconds"])
            finally:
                WATER_PUMP.turnOffPump()
                _publish(pump_active=False)
            self.watered_count += 1
            print(f"💧 Auto mode: watered at row {self.row + 1}")
        else:
            print(f"🌱 Auto mode: soil already wet at row {self.row + 1}")
        self.row_since_check = 0.0
        self._publish_progress(force=True)

    def _path_clear(self):
        """Sample the ultrasonic three times; path is clear only when fewer
        than OBSTACLE_CLEAR_THRESHOLD samples are blocked.

        This is symmetric with the drive-loop confirmation (ULTRASONIC_CONFIRM_SAMPLES):
        a single stale or clean reading cannot prematurely un-block the path.
        Unreadable samples (None or below the minimum valid distance) are
        treated as clear to avoid false positives from sensor glitches.
        """
        if not is_ultrasonic_enabled():
            return True
        blocked = 0
        for _ in range(3):
            distance_cm = get_distance_cm()
            self._sleep(OBSTACLE_SAMPLE_S)
            if distance_cm is None or distance_cm < ULTRASONIC_MIN_VALID_CM:
                continue
            if distance_cm <= ULTRASONIC_THRESHOLD_CM:
                blocked += 1
        return blocked < OBSTACLE_CLEAR_THRESHOLD

    def _sidestep(self, row_target_rad):
        """One perpendicular detour, then line back up on the row."""
        detour_heading = auto_plan.wrap_rad(
            self._heading() + self.turn_sign * (math.pi / 2.0)
        )
        self._set_state(AVOID, "sidestepping the obstacle")
        self._turn_to(detour_heading)
        reason = self._drive(
            SIDESTEP_M,
            detour_heading,
            SIDESTEP_M / MIN_ROW_SPEED_MPS + STEP_TIMEOUT_PAD_S,
            track_row=False,
        )
        if reason == "obstacle":
            raise _AutoFault("obstacle while sidestepping")
        self._turn_to(row_target_rad)

    def _avoid(self, row_target_rad):
        """Wait for the obstacle to clear, then sidestep once, then fault."""
        self._set_state(AVOID, "obstacle detected")
        self._halt()
        self.obstacles += 1
        self._publish_progress(force=True)
        for _ in range(OBSTACLE_MAX_RETRIES):
            self._sleep(OBSTACLE_WAIT_S)
            if self._path_clear():
                return
        if not self.sidestep_used:
            self.sidestep_used = True
            self._sidestep(row_target_rad)
            if self._path_clear():
                self.sidestep_used = False
                return
        raise _AutoFault("path blocked after retries")

    # -------------------------------------------------------------- mission --
    def run(self):
        global _thread, _active_run
        try:
            self._calibrate()
            for row in range(self.total_rows):
                self.row = row
                self.row_distance = 0.0
                self.row_since_check = 0.0
                self.sidestep_used = False
                row_relative = auto_plan.row_relative_deg(row)
                row_target = self._abs(row_relative)
                self._set_state(
                    DRIVE_ROW, f"row {row + 1} of {self.total_rows}"
                )

                while self.row_distance < self.length:
                    remaining = self.length - self.row_distance
                    timeout = remaining / MIN_ROW_SPEED_MPS + ROW_TIMEOUT_PAD_S
                    reason = self._drive(remaining, row_target, timeout)
                    if reason == "water":
                        self._water()
                        self._set_state(
                            DRIVE_ROW, f"row {row + 1} of {self.total_rows}"
                        )
                    elif reason == "obstacle":
                        self._avoid(row_target)
                        self._set_state(
                            DRIVE_ROW, f"row {row + 1} of {self.total_rows}"
                        )

                if row == self.total_rows - 1:
                    break

                # TURN_1 -> STEP across -> TURN_2, both turns the same way.
                step_relative = row_relative + 90.0 * self.turn_sign
                self._set_state(TURN_1, f"turning after row {row + 1}")
                self._turn_to(self._abs(step_relative))
                self._set_state(STEP, f"stepping {self.spacing:.2f} m across")
                self._drive(
                    self.spacing,
                    self._abs(step_relative),
                    self.spacing / MIN_ROW_SPEED_MPS + STEP_TIMEOUT_PAD_S,
                    track_row=False,
                )
                self._set_state(TURN_2, f"lining up row {row + 2}")
                self._turn_to(self._abs(row_relative + 180.0 * self.turn_sign))

            self._set_state(DONE, "sweep complete")
        except _StopRequested:
            self._halt()
            self._set_state(IDLE, f"stopped: {self.stop_reason}")
        except _AutoFault as exc:
            self._halt()
            _publish(fault=str(exc))
            self._set_state(ERROR, str(exc))
            print(f"🛑 Auto mode error: {exc}")
        except Exception as exc:  # never leave a mission thread running blind
            self._halt()
            _publish(fault=str(exc))
            self._set_state(ERROR, f"unexpected error: {exc}")
            print(f"🛑 Auto mode unexpected error: {exc}")
        finally:
            self._halt()
            with _lock:
                _thread = None
                _active_run = None
                _status["active"] = False
            print(f"🚜 Auto mode finished in state {self.state}")

    def request_stop(self, reason="stop requested"):
        self.stop_reason = reason
        self.stop_event.set()


# --------------------------------------------------------------- public API --
def start(params=None):
    """Validate ``params`` and start the mission thread.

    Returns a dict: ``{"success": bool, ...status}``.  Never blocks and never
    touches the motors itself, so it is safe to call from a Flask route.
    """
    global _thread, _active_run
    try:
        clean = _normalise_params(params)
    except (TypeError, ValueError) as exc:
        return {"success": False, "error": str(exc), **get_status()}

    if not get_imu_snapshot().get("available"):
        return {
            "success": False,
            "error": "MPU6050 not available — auto mode needs gyro turns",
            **get_status(),
        }
    if encoders.LEFT_ENCODER is None or encoders.RIGHT_ENCODER is None:
        return {
            "success": False,
            "error": "wheel encoders unavailable — check pigpiod",
            **get_status(),
        }

    with _lock:
        if _thread is not None and _thread.is_alive():
            return {"success": False, "error": "auto mode already running", **get_status()}
        run = _Sweep(clean)
        _active_run = run
        _status.update(
            {
                "active": True,
                "state": CALIBRATE,
                "row": 0,
                "total_rows": run.total_rows,
                "progress_pct": 0.0,
                "heading_error_deg": 0.0,
                "row_distance_m": 0.0,
                "distance_m": 0.0,
                "last_moisture": "not checked",
                "last_moisture_at": None,
                "watered_count": 0,
                "obstacles": 0,
                "pump_active": False,
                "ultrasonic_active": is_ultrasonic_enabled(),
                "reference_heading_deg": None,
                "message": "starting",
                "fault": "",
                "started_at": time.time(),
                "elapsed_s": 0.0,
                "params": dict(clean),
            }
        )
        _thread = threading.Thread(
            target=run.run, name="auto-sweep", daemon=True
        )
        _thread.start()

    message = "auto mode started"
    if clean["drive_speed"] < MIN_DRIVE_DUTY:
        message += (
            f" (drive_speed raised to the {MIN_DRIVE_DUTY:.2f} stall floor"
            " while driving)"
        )
    return {"success": True, "message": message, **get_status()}


def stop(reason="stop requested"):
    """Ask the mission thread to stop and wait briefly for it to halt."""
    reason = str(reason or "stop requested")
    with _lock:
        run = _active_run
        thread = _thread
    if run is None:
        return False
    run.request_stop(reason)
    if (
        thread is not None
        and thread.is_alive()
        and thread is not threading.current_thread()
    ):
        thread.join(STOP_JOIN_TIMEOUT_S)
    return True


def is_active():
    with _lock:
        return bool(_thread is not None and _thread.is_alive())


def get_status():
    """Snapshot of the mission for telemetry: state, row, progress, etc."""
    with _lock:
        status = dict(_status)
        status["active"] = bool(_thread is not None and _thread.is_alive())
        started = status.get("started_at")
        if started and status["active"]:
            status["elapsed_s"] = round(time.time() - started, 1)
        status["ticks_per_metre"] = round(TICKS_PER_METRE, 1)
        status["stop_on_disconnect"] = AUTO_STOP_ON_DISCONNECT
        status["state_name"] = status.get("state", IDLE)
    try:
        status["pump_active"] = WATER_PUMP.isPumpOn()
    except Exception:
        pass
    try:
        status["ultrasonic_active"] = is_ultrasonic_enabled()
    except Exception:
        pass
    return status


__all__ = [
    "IDLE",
    "CALIBRATE",
    "DRIVE_ROW",
    "TURN_1",
    "STEP",
    "TURN_2",
    "WATER",
    "AVOID",
    "DONE",
    "ERROR",
    "DEFAULT_PARAMS",
    "AUTO_STOP_ON_DISCONNECT",
    "METRES_PER_TICK",
    "TICKS_PER_METRE",
    "start",
    "stop",
    "is_active",
    "get_status",
]
