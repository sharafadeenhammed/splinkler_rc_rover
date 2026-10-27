import os
import sys
import signal
import time
import threading
import subprocess
import json
import select
import math
import atexit
import functools
import gc
import hmac

# Line-buffer stdout: under systemd stdout is a journald socket and print()
# output stays block-buffered for hours, hiding the memory watchdog's alerts
# exactly when the operator needs to see them.  TTY (manual) runs unaffected.
try:
    sys.stdout.reconfigure(line_buffering=True)
except (AttributeError, ValueError, OSError):
    pass

# When launched manually (`python main.py`) there is no systemd unit to set
# OOMScoreAdjust=200, so on a global OOM the kernel may kill wpa_supplicant /
# dhcpcd first — or brcmfmac simply starves — and Wi-Fi dies until a power
# cycle.  Raising our own OOM score needs no root, so in a manual run the
# rover is always killed first and the network stack survives.  Under systemd
# INVOCATION_ID is set and the unit's OOMScoreAdjust owns this setting.
if "INVOCATION_ID" not in os.environ:
    try:
        with open("/proc/self/oom_score_adj", "w") as _oom_handle:
            _oom_handle.write("500\n")
        with open("/proc/self/oom_score_adj", "r") as _oom_handle:
            _oom_value = _oom_handle.read().strip()
        print(f"🛡️ Manual run: oom_score_adj=+{_oom_value} (rover dies first on OOM, Wi-Fi survives)")
    except OSError as _oom_err:
        print(f"⚠️ Could not raise oom_score_adj: {_oom_err}")

from flask import Flask, Response, render_template, request, jsonify

# Run at lower priority so SSH / Wi-Fi always get CPU first on the single-core Zero W.
try:
    os.nice(10)
except (OSError, AttributeError):
    pass

try:
    from flask_sock import Sock
    import simple_websocket
    HAS_SOCK = True
except ImportError:
    HAS_SOCK = False
    Sock = None
    simple_websocket = None
    print("⚠️ flask-sock not installed. WebSocket functionality will be disabled.")

from helpers import (
    forward, reverse, turnRight, turnLeft, stop,
    readHumidityAndTemp, camera_servo, sensor_servo,
    MOISTURE_SENSOR, WATER_PUMP,
    get_distance_cm,
    enable_ultrasonic, disable_ultrasonic, toggle_ultrasonic, is_ultrasonic_enabled,
    start_auto_mode, stop_auto_mode, toggle_auto_mode, is_auto_active,
    get_auto_progress, set_auto_speed,
    set_area_polygon,
    get_encoder_ticks, reset_encoders, encoder_config
)
from helpers.mpu6050 import (
    get_imu_snapshot, update_imu, calibrate_imu, reset_imu_heading, set_imu_heading_rad,
    apply_heading_correction,
)
from helpers.hmc5883l import (
    get_compass_snapshot, update_compass, compass_drift_correction, reset_compass_fusion,
)
from helpers.pose_fusion import gps_heading_usable
from helpers import motor_controls
from helpers.gps_controller import get_gps
from helpers.navigation_database import DB, PoseTracker
from helpers.auto_controller import WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV

app = Flask(__name__)
sock = Sock(app) if HAS_SOCK else None

# Optional command PIN: the service environment's ROVER_PIN wins; when it is
# absent, the fallback below applies (an explicit empty value keeps the old
# open behaviour).  The six control endpoints (drive, auto, probe, pump,
# camera pan, auto speed) and their WebSocket equivalents demand it.  The
# dashboard reads pin_required from telemetry and shows the PIN field only
# when it is enforced.
ROVER_PIN = os.environ.get("ROVER_PIN", "rover_123").strip()
_PROTECTED_WS_COMMANDS = {"motor", "sensor_deploy", "pump", "auto_mode", "auto_speed", "camera_tilt"}


def _pin_ok(supplied=None):
    if not ROVER_PIN:
        return True
    if supplied is None:  # HTTP route: header first, query param as fallback
        supplied = request.headers.get("X-Rover-Pin", "") or request.args.get("pin", "")
    return hmac.compare_digest(str(supplied).strip(), ROVER_PIN)


def pin_required(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        if not _pin_ok():
            return jsonify({"success": False, "error": "PIN required"}), 401
        return fn(*args, **kwargs)
    return wrapper

camera_current_angle = 0
sensor_servo_deployed = False
motor_status = "STOPPED"
sensor_data = {
  "temperature": 0.0,
  "humidity": 0.0,
  "soil":"dry",
  "last_updated": "Never"
}
data_lock = threading.Lock()
clients_lock = threading.Lock()
connected_clients = set()
MAX_WS_CLIENTS = 2
pose_tracker = PoseTracker(WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV)
saved_area = DB.active_area()
saved_map_origin = DB.map_origin()
_map_origin_cache = {"value": saved_map_origin}
if saved_map_origin:
    pose_tracker.set_map_origin(saved_map_origin[0], saved_map_origin[1])
if saved_area and len(saved_area.get("polygon", [])) >= 3:
    try:
        set_area_polygon(saved_area["polygon"])
    except (TypeError, ValueError):
        pass

# Cache for the active work area so telemetry doesn't re-read the store every build.
_area_cache = {"value": None, "ts": 0.0}


def cached_active_area():
    now = time.time()
    if now - _area_cache["ts"] > 5.0:
        _area_cache["value"] = DB.active_area()
        _area_cache["ts"] = now
    return _area_cache["value"]


def navigation_loop():
  """Update local pose at 20 Hz (nothing is persisted)."""
  print("🧭 Navigation loop started.")
  while True:
    try:
      compass = update_compass()
      imu = update_imu()
      if compass.get("available") and imu.get("available"):
        correction = compass_drift_correction(imu["heading_rad"])
        if correction:
          apply_heading_correction(correction)
          imu = get_imu_snapshot()
      left_ticks, right_ticks = get_encoder_ticks()
      pose_tracker.update(
          left_ticks, right_ticks, imu,
          gps=get_gps().get(),
          map_origin=_map_origin_cache.get("value"),
      )
      time.sleep(0.05)
    except Exception as exc:
      print(f"⚠️ Navigation update error: {exc}")
      time.sleep(1.0)

def read_sensor_loop():
  print("🛰️ Sensor background thread successfully started.")
  while True:
    try:
      # 1. Fetch hardware telemetry data
      temp, humidity = readHumidityAndTemp()
      # if temp or humidity: print("temp: ", temp, "  humidity: ", humidity)
            
      # 2. Safely commit the new values to memory using a lock
      with data_lock:
        if temp: sensor_data["temperature"] = temp
        if humidity: sensor_data["humidity"] = humidity
        if temp or humidity: sensor_data["last_updated"] = time.strftime("%y-%m-%d %H:%M:%S")
      time.sleep(1.0)
    except Exception as e:
      print(f"⚠️ Sensor read error: {e}")
      # Sensor chips (like DHT22 or BME280) should not be polled faster than 1-2 seconds.
      time.sleep(2.0)

class SharedCameraStream:
    """One low-bitrate camera process shared by the dashboard stream."""

    def __init__(self):
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.process = None
        self.clients = 0
        self.sequence = 0
        self.frame = None  # Keep only the latest JPEG in RAM.
        self.last_client_at = time.monotonic()
        self.blocked_until = 0.0  # memory watchdog cooldown (monotonic)

    def subscribe(self):
        with self.lock:
            if time.monotonic() < self.blocked_until:
                return False
            if self.clients >= 1:
                return False
            self.clients = 1
            self.last_client_at = time.monotonic()
            if self.process is None or self.process.poll() is not None:
                self._start_locked()
            return True

    def unsubscribe(self):
        with self.condition:
            self.clients = 0
            self.last_client_at = time.monotonic()
            self.condition.notify_all()

    def stop(self, reason="idle", cooldown_sec=0.0):
        """Kill the camera process now (memory pressure) and optionally refuse
        to restart for cooldown_sec — the dashboard retries /stream on a timer,
        so without a cooldown the camera would thrash straight back on."""
        with self.condition:
            self.clients = 0
            self.last_client_at = 0.0  # reader thread sees idle immediately
            if cooldown_sec:
                self.blocked_until = time.monotonic() + cooldown_sec
            print(f"📷 Camera stopped ({reason}).")
            self.condition.notify_all()

    def _start_locked(self):
        command = [
            "rpicam-vid", "-t", "0", "--codec", "mjpeg", "--quality", "50",
            "--inline", "-o", "-", "--width", "640", "--height", "480",
            "--framerate", "4", "-n", "--denoise", "cdn_off",
        ]
        try:
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
            )
        except (FileNotFoundError, OSError) as exc:
            self.process = None
            print(f"Camera unavailable: {exc}")
            return
        threading.Thread(target=self._reader, args=(self.process,), daemon=True).start()

    def _reader(self, process):
        buffer = bytearray()
        try:
            while process.poll() is None:
                with self.lock:
                    idle = (self.clients == 0
                            and time.monotonic() - self.last_client_at > 5.0)
                if idle:
                    break
                ready, _, _ = select.select([process.stdout], [], [], 0.5)
                if not ready:
                    continue
                chunk = os.read(process.stdout.fileno(), 8192)
                if not chunk:
                    break
                buffer.extend(chunk)
                if len(buffer) > 128 * 1024:
                    del buffer[:-8192]
                while True:
                    start = buffer.find(b"\xff\xd8")
                    if start < 0:
                        buffer.clear()
                        break
                    end = buffer.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start:
                            del buffer[:start]
                        break
                    frame = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]
                    with self.condition:
                        self.frame = frame
                        self.sequence += 1
                        self.condition.notify_all()
        except (OSError, ValueError) as exc:
            print(f"Camera stream ended: {exc}")
        finally:
            try:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
            with self.condition:
                if self.process is process:
                    self.process = None
                    self.frame = None
                    self.sequence += 1
                self.condition.notify_all()

    def frames(self):
        if not self.subscribe():
            return
        last_sequence = -1
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.sequence != last_sequence or self.clients == 0,
                        timeout=2.0,
                    )
                    if self.clients == 0:
                        break
                    frame = self.frame
                    sequence = self.sequence
                if frame is not None and sequence != last_sequence:
                    last_sequence = sequence
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                           str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n")
        finally:
            self.unsubscribe()


camera_stream = SharedCameraStream()

class WSClientWrapper:
    def __init__(self, ws):
        self.ws = ws
        self.lock = threading.Lock()

    def send(self, data):
        with self.lock:
            self.ws.send(data)

def handle_motor(speed, direction):
    global motor_status
    if is_auto_active():
        stop_auto_mode()

    speed_percent = max(0, min(100, int(speed or 0)))
    duty_cycle = speed_percent / 100.0

    if speed_percent == 0 or direction == 'stop':
        stop()
        motor_status = "STOPPED"
    elif direction == 'forward':
        forward(duty_cycle)
        motor_status = f"FORWARD ({speed_percent}%)"
    elif direction == 'reverse':
        reverse(duty_cycle)
        motor_status = f"REVERSE ({speed_percent}%)"
    elif direction == 'right':
        turnRight(duty_cycle)
        motor_status = f"TURNING RIGHT ({speed_percent}%)"
    elif direction == 'left':
        turnLeft(duty_cycle)
        motor_status = f"TURNING LEFT ({speed_percent}%)"
    else:
        stop()
        motor_status = "STOPPED"

    print(motor_status)
    return motor_status

def handle_sensor_deploy(action=None):
    global sensor_servo_deployed
    if action == "deploy" or (action == "toggle" and not sensor_servo_deployed):
        sensor_servo.angle = -20
        sensor_servo_deployed = True
    elif action == "retract" or (action == "toggle" and sensor_servo_deployed):
        sensor_servo.angle = -90
        sensor_servo_deployed = False
    elif not action or action == "toggle":
        sensor_servo_deployed = not sensor_servo_deployed
        sensor_servo.angle = -20 if sensor_servo_deployed else -90
    time.sleep(0.1)
    return {
        "is_deployed": sensor_servo_deployed,
        "angle": sensor_servo.angle,
        "status": "DEPLOYED (-20°)" if sensor_servo_deployed else "RETRACTED (-90°)"
    }

def handle_water_pump(action=None):
    if action == "on":
        is_active = WATER_PUMP.turnOnPump()
    elif action == "off":
        is_active = WATER_PUMP.turnOffPump()
    elif action == "toggle" or not action:
        is_active = WATER_PUMP.togglePump()
    elif action == "status":
        is_active = WATER_PUMP.isPumpOn()
    else:
        is_active = WATER_PUMP.isPumpOn()
    return {
        "pump_active": is_active,
        "status": "RUNNING 💧" if is_active else "OFF ⏹️"
    }

def handle_auto_mode(action=None):
    if action == "start":
        active = start_auto_mode()
    elif action == "stop":
        active = stop_auto_mode()
    elif action == "toggle" or not action:
        active = toggle_auto_mode()
    elif action == "status":
        active = is_auto_active()
    else:
        active = is_auto_active()
    return {
        "auto_active": active,
        "status": "AUTONOMOUS 🤖" if active else "MANUAL 🎮"
    }

def handle_ultrasonic(action=None):
    if action == "enable":
        enabled = enable_ultrasonic()
    elif action == "disable":
        enabled = disable_ultrasonic()
    elif action == "toggle" or not action:
        enabled = toggle_ultrasonic()
    elif action == "status":
        enabled = is_ultrasonic_enabled()
    else:
        enabled = is_ultrasonic_enabled()
    return {
        "ultrasonic_enabled": enabled,
        "status": "ENABLED 🔊" if enabled else "DISABLED 🔇"
    }

def handle_camera_tilt(direction):
    global camera_current_angle
    step_degrees = 15
    if direction == "left":
        camera_current_angle = max(-90, camera_current_angle + step_degrees)
    elif direction == "right":
        camera_current_angle = min(90, camera_current_angle - step_degrees)
    elif direction == "center":
        camera_current_angle = 0
    try:
        camera_servo.angle = camera_current_angle
        time.sleep(0.1)
    except Exception as e:
        print(f"⚠️ Camera tilt hardware error: {e}")
    return camera_current_angle

def get_telemetry_payload():
    with data_lock:
        payload = sensor_data.copy()
    payload["soil"] = MOISTURE_SENSOR.getSoilMoistureState()
    payload["pump_active"] = WATER_PUMP.isPumpOn()
    payload["sensor_deployed"] = sensor_servo_deployed or (sensor_servo.angle is not None and sensor_servo.angle > -50)
    payload["sensor_angle"] = sensor_servo.angle
    payload["distance_cm"] = get_distance_cm()
    payload["ultrasonic_enabled"] = is_ultrasonic_enabled()
    payload["auto_active"] = is_auto_active()
    payload["auto_progress"] = get_auto_progress()
    payload["motor_status"] = motor_status
    payload["camera_angle"] = camera_current_angle
    lt, rt = get_encoder_ticks()
    payload["encoder_left"] = lt
    payload["encoder_right"] = rt
    # Live wheel duty (−1..1, logical direction) for the dashboard drive bars.
    left_duty = motor_controls.last_left_speed or 0.0
    right_duty = motor_controls.last_right_speed or 0.0
    if motor_controls.MOTOR_REVERSED:
        left_duty, right_duty = -left_duty, -right_duty
    payload["motor_drive"] = {"left": float(left_duty), "right": float(right_duty)}
    payload["encoder_config"] = encoder_config()
    payload["imu"] = get_imu_snapshot()
    payload["compass"] = get_compass_snapshot()
    payload["pose"] = pose_tracker.snapshot()
    payload["active_area"] = cached_active_area()
    payload["map_origin"] = _map_origin_cache["value"]
    payload["gps"] = get_gps().get()
    payload["memory"] = read_meminfo()
    payload["pin_required"] = bool(ROVER_PIN)
    payload["success"] = True
    return payload


def apply_navigation_origin(latitude, longitude, source="manual"):
    """Reset local pose so this lat/lon becomes map centre (0, 0 m)."""
    global _map_origin_cache
    if not (-85.0 <= latitude <= 85.0 and -180.0 <= longitude <= 180.0):
        raise ValueError("Map origin is outside supported coordinates")
    if is_auto_active() or motor_status != "STOPPED":
        raise RuntimeError("Stop the rover before changing its map start")
    _map_origin_cache["value"] = DB.save_map_origin(latitude, longitude)
    pose_tracker.set_map_origin(latitude, longitude)
    heading_rad = 0.0
    snap = get_gps().get()
    if gps_heading_usable(snap):
        heading_rad = math.radians(float(snap["course_deg"]))
    elif source == "gps" and snap.get("fix"):
        heading_rad = 0.0
    pose_tracker.reset(0.0, 0.0, heading_rad)
    set_imu_heading_rad(heading_rad)
    reset_compass_fusion()
    return {
        "map_origin": [latitude, longitude],
        "source": source,
        "heading_deg": math.degrees(heading_rad),
        "gps": snap,
    }


def handle_set_auto_speed(speed_pct):
    """Set autonomous cruise speed (0–100). Persists across auto mode restarts."""
    try:
        spd = max(10, min(100, int(float(speed_pct))))
        set_auto_speed(spd)
        return {"success": True, "auto_speed_pct": spd}
    except (TypeError, ValueError) as e:
        return {"success": False, "error": str(e)}


def emergency_stop_rover(reason="user disconnected"):
    global motor_status, sensor_servo_deployed
    print(f"🛑 [SAFETY FAILSAFE] Stopping all rover actions ({reason})...")
    # 1. Stop motors
    try:
        stop()
        motor_status = "STOPPED"
    except Exception as e:
        print(f"⚠️ Failsafe motor stop error: {e}")

    # 2. Stop water pump
    try:
        WATER_PUMP.turnOffPump()
    except Exception as e:
        print(f"⚠️ Failsafe pump stop error: {e}")

    # 3. Disengage autonomous mode if running
    try:
        if is_auto_active():
            stop_auto_mode()
    except Exception as e:
        print(f"⚠️ Failsafe auto mode stop error: {e}")

    # 4. Retract sensor arm to safe home position (-90 deg)
    try:
        sensor_servo.angle = -90
        sensor_servo_deployed = False
    except Exception as e:
        print(f"⚠️ Failsafe servo retract error: {e}")

# ----------------- Memory watchdog (Pi Zero W, 512 MB total) ----------------
# The rover shares 512 MB with the Wi-Fi firmware.  When RAM runs out the
# brcmfmac driver fails to allocate its buffers and Wi-Fi dies until a power
# cycle, stranding the operator.  This watchdog sheds load (camera first,
# then motors/pump/auto) BEFORE the kernel gets desperate.
MEMINFO_PATH = "/proc/meminfo"
MEM_WARN_MB = float(os.environ.get("MEM_WARN_MB", "80"))
MEM_CRITICAL_MB = float(os.environ.get("MEM_CRITICAL_MB", "60"))
# Breadcrumb tier above the shed threshold: garbage collection + 1 s polling
# start here so a fast leak is caught before it reaches warn/critical.
MEM_EARLY_MB = float(os.environ.get("MEM_EARLY_MB", "150"))
WATCHDOG_TICK_SLOW = 5.0
WATCHDOG_TICK_FAST = 1.0
# Stop the rover this long after the operator's network link disappears.
LINK_DOWN_STOP_SEC = float(os.environ.get("LINK_DOWN_STOP_SEC", "10"))
WIFI_OPERSTATE_PATH = "/sys/class/net/wlan0/operstate"
WIFI_ROUTE_PATH = "/proc/net/route"
_mem_warned = False
_mem_early_logged = False
_hist_dumped = False
_mem_history = []


def read_meminfo(path=MEMINFO_PATH):
    """Parse /proc/meminfo -> {"mem_total_mb", "mem_available_mb"} or None."""
    try:
        values = {}
        with open(path, "r") as handle:
            for line in handle:
                key, sep, rest = line.partition(":")
                if not sep:
                    continue
                parts = rest.split()
                if parts:
                    values[key] = float(parts[0]) / 1024.0  # kB -> MB
        if "MemAvailable" not in values:
            return None
        swap_total = values.get("SwapTotal", 0.0)
        return {
            "mem_total_mb": values.get("MemTotal"),
            "mem_available_mb": values["MemAvailable"],
            "swap_total_mb": swap_total,
            "swap_used_mb": max(0.0, swap_total - values.get("SwapFree", 0.0)),
        }
    except (OSError, ValueError):
        return None


def read_self_rss_mb():
    """Resident set size of this process in MB (None if unreadable)."""
    try:
        with open("/proc/self/status", "r") as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    return float(line.split()[1]) / 1024.0
    except (OSError, ValueError, IndexError):
        return None
    return None


def wifi_link_state(operstate_path=WIFI_OPERSTATE_PATH, route_path=WIFI_ROUTE_PATH):
    """Operator link health: 'up' (a default route exists — the operator can
    still reach the rover over any interface), 'down' (no default route but a
    Wi-Fi interface exists) or 'absent' (no interface — guard disabled)."""
    try:
        with open(route_path, "r") as handle:
            handle.readline()  # header
            for line in handle:
                parts = line.split()
                if len(parts) >= 4 and parts[1] == "00000000":
                    try:
                        if int(parts[3], 16) & 0x1:
                            return "up"
                    except ValueError:
                        continue
    except OSError:
        return "absent"
    if not os.path.exists(operstate_path):
        return "absent"
    return "down"


def _dump_mem_history():
    """Print recent watchdog samples so the journal shows the ramp that led
    to a shed/critical event (previous incident logs were lost to reboot)."""
    if not _mem_history:
        return
    print(f"📟 Memory history ({len(_mem_history)} samples, oldest first):")
    for entry in _mem_history:
        print(f"   {entry}")


def memory_watchdog_loop():
    """Shed load on low memory and stop the rover if the operator link dies.
    Runs for manual AND systemd launches: the unit's MemoryMax/OOMScoreAdjust
    only exist under systemd, so this in-process guard plus the startup
    oom_score_adj raise are what protect a bare `python main.py` run."""
    global _mem_warned, _mem_early_logged, _hist_dumped
    print("🧠 Memory watchdog started.")
    link = wifi_link_state()
    if link == "absent":
        print("🔎 Wi-Fi link monitor disabled (no network interface found).")
    link_down_since = None
    link_down_logged = False
    link_stop_done = False
    tick = WATCHDOG_TICK_SLOW
    while True:
        try:
            time.sleep(tick)
            mem = read_meminfo()
            if mem is None:
                continue
            avail = mem["mem_available_mb"]
            rss = read_self_rss_mb()
            rss_txt = f", rover RSS {rss:.0f} MB" if rss is not None else ""
            _mem_history.append(
                f"{time.strftime('%H:%M:%S')} avail={avail:.0f}MB "
                f"rss={rss if rss is not None else '?'}MB "
                f"swap={mem.get('swap_used_mb', 0.0):.0f}MB"
            )
            del _mem_history[:-90]
            tick = WATCHDOG_TICK_FAST if avail < MEM_EARLY_MB else WATCHDOG_TICK_SLOW

            # Early breadcrumb: start reclaiming before the shed threshold.
            if avail < MEM_EARLY_MB and not _mem_early_logged:
                print(f"🔎 MEMORY WATCH: {avail:.0f} MB available{rss_txt} - collecting garbage")
                gc.collect()
                _mem_early_logged = True
            elif avail > MEM_EARLY_MB + 20:
                _mem_early_logged = False

            # Camera is the biggest optional consumer: shed it first.  Also
            # shed again if the dashboard reconnected it while still low.
            if avail < MEM_WARN_MB and (not _mem_warned or camera_stream.clients >= 1):
                print(f"⚠️ LOW MEMORY: {avail:.0f} MB available{rss_txt} - shedding camera stream")
                if not _hist_dumped:
                    _dump_mem_history()
                    _hist_dumped = True
                camera_stream.stop("low memory", cooldown_sec=60.0)
                gc.collect()

            # Never let the rover keep driving on a starved board.
            if (avail < MEM_CRITICAL_MB and
                    (is_auto_active() or motor_status != "STOPPED" or WATER_PUMP.isPumpOn())):
                print(f"🛑 CRITICAL MEMORY: {avail:.0f} MB available{rss_txt} - emergency stop")
                _dump_mem_history()
                emergency_stop_rover(f"low memory ({avail:.0f} MB)")
                camera_stream.stop("memory critical", cooldown_sec=120.0)
                gc.collect()

            _mem_warned = avail < MEM_WARN_MB
            if avail >= MEM_WARN_MB:
                _hist_dumped = False

            # Operator link guard: Wi-Fi death used to strand a moving rover
            # with no kill switch (brcmfmac starves first under memory
            # pressure).  Stop everything once the link is long gone.
            if link != "absent":
                now = time.monotonic()
                if wifi_link_state() == "down":
                    if link_down_since is None:
                        link_down_since = now
                    elapsed = now - link_down_since
                    moving = is_auto_active() or motor_status != "STOPPED" or WATER_PUMP.isPumpOn()
                    if elapsed >= LINK_DOWN_STOP_SEC and not link_stop_done:
                        if moving:
                            print(f"📡 WI-FI LINK LOST {elapsed:.0f}s - emergency stop (operator cannot reach the rover)")
                            emergency_stop_rover("Wi-Fi link lost")
                            link_stop_done = True
                        elif not link_down_logged:
                            print(f"📡 Wi-Fi link down {elapsed:.0f}s (rover idle, waiting for recovery)")
                            link_down_logged = True
                elif link_down_since is not None:
                    if now - link_down_since >= 3.0:
                        print(f"✅ Wi-Fi link restored after {now - link_down_since:.0f}s")
                    link_down_since = None
                    link_down_logged = False
                    link_stop_done = False
        except Exception as exc:
            print(f"⚠️ Memory watchdog error: {exc}")
            time.sleep(WATCHDOG_TICK_SLOW)


def _atexit_stop():
    """Cut motors/pump on any orderly exit (Ctrl-C, systemctl stop, crash)."""
    try:
        emergency_stop_rover("process exiting")
    except Exception:
        pass


def _sigterm_handler(signum, frame):
    """systemd stop/restart: stop hardware, then exit so atexit can finish."""
    try:
        emergency_stop_rover(f"signal {signum}")
    except Exception:
        pass
    sys.exit(0)


def broadcast_telemetry(payload=None):
    if not HAS_SOCK:
        return
    if payload is None:
        payload = get_telemetry_payload()
    msg = json.dumps({"type": "telemetry", "data": payload})
    with clients_lock:
        clients_copy = list(connected_clients)

    for client in clients_copy:
        try:
            client.send(msg)
        except Exception:
            with clients_lock:
                connected_clients.discard(client)

def ws_telemetry_broadcast_loop():
    print("🛰️ WS Telemetry broadcast thread started.")
    while True:
        try:
            time.sleep(1.0)
            with clients_lock:
                if not connected_clients:
                    continue
            broadcast_telemetry()
        except Exception as e:
            time.sleep(1.0)

# Background loops start exactly once, at module import — covering both
# `python main.py` and a WSGI server (gunicorn imports main:app without ever
# running __main__), so telemetry needs no HTTP request to exist.
# NOTE: the camera vision-analysis pipeline was removed entirely (CPU/memory
# load on the Pi Zero W); obstacle detection is ultrasonic-only now.
_threads_lock = threading.Lock()

def start_background_threads():
    with _threads_lock:
        if not getattr(app, 'sensor_thread_started', False):
            bg_thread = threading.Thread(target=read_sensor_loop, daemon=True)
            bg_thread.start()
            app.sensor_thread_started = True

        if not getattr(app, 'navigation_thread_started', False):
            nav_thread = threading.Thread(target=navigation_loop, daemon=True)
            nav_thread.start()
            app.navigation_thread_started = True

        if not getattr(app, 'memory_watchdog_started', False):
            mem_thread = threading.Thread(target=memory_watchdog_loop, daemon=True)
            mem_thread.start()
            app.memory_watchdog_started = True

        if HAS_SOCK and not getattr(app, 'ws_broadcast_started', False):
            ws_thread = threading.Thread(target=ws_telemetry_broadcast_loop, daemon=True)
            ws_thread.start()
            app.ws_broadcast_started = True


@app.after_request
def set_static_cache_headers(response):
    """Cache static assets for a day: over the rover's flaky Wi-Fi the
    dashboard was re-downloading css/js on every single page load."""
    if request.path.startswith("/static/"):
        # Overwrite (not setdefault): Flask's default is `no-cache`, which
        # would otherwise win and keep the css/js re-downloading every load.
        response.headers["Cache-Control"] = "public, max-age=86400"
    return response

# WebSocket Endpoint
if HAS_SOCK:
    @sock.route('/ws')
    def ws_handler(ws):
        with clients_lock:
            if len(connected_clients) >= MAX_WS_CLIENTS:
                try:
                    ws.close()
                except Exception:
                    pass
                return
        client = WSClientWrapper(ws)
        with clients_lock:
            connected_clients.add(client)
        print(f"⚡ WebSocket client connected. Active clients: {len(connected_clients)}")

        # Send immediate initial telemetry and connection acknowledgment to client
        try:
            client.send(json.dumps({
                "type": "connection_ack",
                "reconnected": True,
                "data": get_telemetry_payload()
            }))
        except Exception:
            pass

        last_activity = time.time()
        # Disconnect sockets that stop sending (the frontend pings every 2 s).
        # 10 s rides out short radio stalls seen in the field without letting a
        # driven rover run blind for long.  Override: WS_HEARTBEAT_TIMEOUT_SEC.
        CLIENT_HEARTBEAT_TIMEOUT_SEC = float(os.environ.get("WS_HEARTBEAT_TIMEOUT_SEC", "10"))
        ping_gap_warned = False

        try:
            while True:
                msg = ws.receive(timeout=0.2)
                now = time.time()

                if msg is None:
                    gap = now - last_activity
                    if gap > CLIENT_HEARTBEAT_TIMEOUT_SEC:
                        print(f"⚠️ WebSocket client silent {gap:.0f}s - dropping connection")
                        break
                    if gap >= 4.0 and not ping_gap_warned:
                        print(f"⚠️ WS ping gap {gap:.0f}s - client messages stalled (radio path?)")
                        ping_gap_warned = True
                    continue

                if ping_gap_warned:
                    print(f"✅ WS messages resumed after {now - last_activity:.0f}s stall")
                    ping_gap_warned = False
                last_activity = now

                try:
                    data = json.loads(msg) if isinstance(msg, str) else json.loads(msg.decode('utf-8'))
                    cmd_type = data.get("type")
                    response = {"type": "response", "success": True, "command": cmd_type}

                    if cmd_type in _PROTECTED_WS_COMMANDS and not _pin_ok(data.get("pin")):
                        response["success"] = False
                        response["error"] = "PIN required"
                        response["message"] = "PIN required — set it in the sidebar"
                        client.send(json.dumps(response))
                        continue

                    if cmd_type == "motor":
                        speed = data.get("speed", 0)
                        direction = data.get("dir", "forward")
                        status = handle_motor(speed, direction)
                        response["status"] = status
                    elif cmd_type == "sensor_deploy":
                        res = handle_sensor_deploy(data.get("action"))
                        response.update(res)
                    elif cmd_type == "pump":
                        res = handle_water_pump(data.get("action"))
                        response.update(res)
                    elif cmd_type == "auto_mode":
                        res = handle_auto_mode(data.get("action"))
                        response.update(res)
                    elif cmd_type == "auto_speed":
                        res = handle_set_auto_speed(data.get("speed_pct", 70))
                        response.update(res)
                    elif cmd_type == "camera_tilt":
                        angle = handle_camera_tilt(data.get("direction"))
                        response["current_angle"] = angle
                    elif cmd_type == "ultrasonic":
                        res = handle_ultrasonic(data.get("action"))
                        response.update(res)
                    elif cmd_type == "get_status":
                        pass
                    elif cmd_type == "ping":
                        response["type"] = "pong"
                    else:
                        response["success"] = False
                        response["message"] = f"Unknown command '{cmd_type}'"

                    response["telemetry"] = get_telemetry_payload()
                    client.send(json.dumps(response))

                    if cmd_type not in ("ping", "get_status"):
                        broadcast_telemetry(response["telemetry"])

                except json.JSONDecodeError:
                    client.send(json.dumps({"type": "error", "message": "Invalid JSON format"}))
                except Exception as ex:
                    client.send(json.dumps({"type": "error", "message": str(ex)}))

        except (simple_websocket.ConnectionClosed, simple_websocket.ConnectionError):
            pass
        except Exception as e:
            print(f"⚠️ WebSocket connection exception: {e}")
        finally:
            with clients_lock:
                connected_clients.discard(client)
                remaining = len(connected_clients)
            print(f"🔌 WebSocket client disconnected. Remaining: {remaining}")

            # Stop all rover actions immediately on user disconnect
            emergency_stop_rover("user disconnected from WebSocket")

            # Broadcast stopped telemetry to any remaining clients
            if remaining > 0:
                broadcast_telemetry()

# ----------------- Existing HTTP Endpoints (Preserved) -----------------

@app.route('/stream')
def video_stream():
    with camera_stream.lock:
        if camera_stream.clients >= 1:
            return Response("Camera stream already in use", status=429, headers={"Retry-After": "3"})
    return Response(
        camera_stream.frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={"Cache-Control": "no-store, no-cache, must-revalidate", "X-Accel-Buffering": "no"},
    )

@app.route("/")
def home():
    return render_template("index.html")

@app.route('/motor', methods=['GET'])
@pin_required
def control_motor():
    try:
        speed = request.args.get('speed', 0)
        direction = request.args.get('dir', 'forward')
        status = handle_motor(speed, direction)
        broadcast_telemetry()
        return jsonify({"success": True, "status": status})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

@app.route("/sensor_readings", methods=['GET'])
def getHumidityReading():
    return jsonify(get_telemetry_payload()), 200

@app.route('/ultrasonic', methods=['GET', 'POST'])
def control_ultrasonic():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_ultrasonic(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/auto_mode', methods=['GET', 'POST'])
@pin_required
def control_auto_mode():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_auto_mode(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/deploy_sensor', methods=['GET', 'POST'])
@pin_required
def deploy_sensor():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_sensor_deploy(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/water_pump', methods=['GET', 'POST'])
@pin_required
def control_pump():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_water_pump(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/camera_tilt', methods=['GET'])
@pin_required
def tilt_camera():
    direction = request.args.get('direction')
    if not direction:
        return jsonify({"status": "error", "message": "Missing 'direction' query parameter."}), 400
    if direction not in ("left", "right", "center"):
        return jsonify({"status": "error", "message": f"Invalid direction '{direction}'. Use 'left', 'right', or 'center'."}), 400

    angle = handle_camera_tilt(direction)
    broadcast_telemetry()
    return jsonify({"status": "success", "current_angle": angle})

@app.route('/auto_speed', methods=['GET', 'POST'])
@pin_required
def set_auto_speed_endpoint():
    speed_pct = (request.args.get('speed_pct') or
                 (request.json.get('speed_pct') if request.is_json else None) or 70)
    res = handle_set_auto_speed(speed_pct)
    broadcast_telemetry()
    return jsonify(res), 200

@app.route('/api/areas', methods=['GET'])
def list_work_areas():
    return jsonify({"success": True, "areas": DB.list_areas()}), 200


@app.route('/api/areas/polygon', methods=['POST'])
def save_polygon_work_area():
    if is_auto_active():
        return jsonify({"success": False, "error": "Stop autonomous mode before changing the work area"}), 409
    payload = request.get_json(silent=True) or {}
    try:
        points = payload.get("points", [])
        map_points = payload.get("map_points", [])
        origin = payload.get("origin")
        if len(points) < 3 or len(points) > 128 or len(map_points) != len(points):
            raise ValueError("Provide 3 to 128 matching map boundary points")
        if any(len(point) != 2 or not all(math.isfinite(float(value)) for value in point) for point in points):
            raise ValueError("Boundary coordinates must be finite x/y values")
        if any(len(point) != 2 or not (-85 <= float(point[0]) <= 85 and -180 <= float(point[1]) <= 180) for point in map_points):
            raise ValueError("Boundary points must be valid latitude/longitude coordinates")
        if not isinstance(origin, (list, tuple)) or len(origin) != 2:
            raise ValueError("Set the rover start on the map before saving the boundary")
        if not (-85 <= float(origin[0]) <= 85 and -180 <= float(origin[1]) <= 180):
            raise ValueError("Rover start must be a valid latitude/longitude coordinate")
        area = DB.save_polygon_area(payload.get("name", "Work area"), points, map_points, origin)
        if origin and len(origin) == 2:
            _map_origin_cache["value"] = DB.save_map_origin(origin[0], origin[1])
        set_area_polygon(points)
        _area_cache["ts"] = 0.0
        return jsonify({"success": True, "area": area, "coverage_mode": "polygon"}), 201
    except (TypeError, ValueError, KeyError, IndexError) as exc:
        return jsonify({"success": False, "error": str(exc)}), 400


@app.route('/api/navigation/origin', methods=['POST'])
def set_navigation_origin():
    payload = request.get_json(silent=True) or {}
    try:
        latitude = float(payload["latitude"])
        longitude = float(payload["longitude"])
        result = apply_navigation_origin(latitude, longitude, source=payload.get("source", "manual"))
        return jsonify({"success": True, **result}), 200
    except RuntimeError as exc:
        return jsonify({"success": False, "error": str(exc)}), 409
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"success": False, "error": str(exc)}), 400


@app.route('/api/navigation/origin/gps', methods=['POST'])
def set_navigation_origin_from_gps():
    snap = get_gps().get()
    if not snap.get("available"):
        return jsonify({"success": False, "error": snap.get("error") or "GPS module not available"}), 503
    if not snap.get("fix"):
        return jsonify({
            "success": False,
            "error": "No GPS fix yet — move outdoors or wait for satellites",
            "gps": snap,
        }), 409
    try:
        result = apply_navigation_origin(float(snap["lat"]), float(snap["lon"]), source="gps")
        return jsonify({"success": True, "gps": snap, **result}), 200
    except RuntimeError as exc:
        return jsonify({"success": False, "error": str(exc)}), 409
    except (TypeError, ValueError) as exc:
        return jsonify({"success": False, "error": str(exc)}), 400


@app.route('/api/areas/<int:area_id>', methods=['GET'])
def get_work_area(area_id):
    area = DB.get_area(area_id)
    if not area:
        return jsonify({"success": False, "error": "Area not found"}), 404
    return jsonify({"success": True, "area": area}), 200


@app.route('/api/imu/calibrate', methods=['POST'])
def calibrate_imu_endpoint():
    if is_auto_active() or motor_status != "STOPPED":
        return jsonify({"success": False, "error": "Stop the rover before calibrating the MPU6050"}), 409
    ok = calibrate_imu()
    reset_imu_heading()
    reset_compass_fusion()
    return jsonify({"success": ok, "imu": get_imu_snapshot(), "compass": get_compass_snapshot()}), (200 if ok else 503)

# Orderly motor/pump stop on every exit path: atexit covers `python main.py`
# and gunicorn's graceful worker stop; if the process is SIGKILLed instead,
# deploy/rover.service's ExecStopPost runs deploy/emergency_stop.py.
atexit.register(_atexit_stop)
start_background_threads()

if __name__ == '__main__':
    # Manual/dev run via Werkzeug.  Production uses gunicorn (see
    # deploy/gunicorn.conf.py), which never executes this block — its worker
    # runs the atexit handlers above on graceful stop instead.
    signal.signal(signal.SIGTERM, _sigterm_handler)
    # host='0.0.0.0' exposes the server to your entire local network (Wi-Fi/LAN)
    # threaded=True allows multiple backend connections without blocking
    # debug=False: debug mode adds overhead and exposes the Werkzeug debugger to the network
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False, use_reloader=False)
