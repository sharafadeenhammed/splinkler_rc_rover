import os
import sys
import signal
import time
import threading
import json
import math
import atexit
import functools
import hmac

from flask import Flask, render_template, request, jsonify
from flask_sock import Sock
import simple_websocket

from helpers import (
    forward, reverse, turnRight, turnLeft, stop,
    readHumidityAndTemp, camera_servo, sensor_servo,
    MOISTURE_SENSOR, WATER_PUMP,
    get_distance_cm,
    enable_ultrasonic, disable_ultrasonic, toggle_ultrasonic, is_ultrasonic_enabled,
    get_encoder_ticks, encoder_config
)
from helpers.mpu6050 import (
    get_imu_snapshot, update_imu, calibrate_imu, reset_imu_heading, set_imu_heading_rad,
    apply_heading_correction,
)
from helpers.hmc5883l import (
    get_compass_snapshot, update_compass, compass_drift_correction, reset_compass_fusion,
)
from helpers.pose_fusion import course_to_heading_rad, gps_heading_usable
from helpers import motor_controls
from helpers import auto_controller
from helpers.gps_controller import get_gps
from helpers.navigation_database import DB, PoseTracker
from helpers.encoder_controller import (
    WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV,
)

app = Flask(__name__)
sock = Sock(app)

# Optional command PIN: the service environment's ROVER_PIN wins; when it is
# absent, the fallback below applies (an explicit empty value keeps the old
# open behaviour).  The four control endpoints (drive, probe, pump, camera
# pan) and their WebSocket equivalents demand it.  The
# dashboard reads pin_required from telemetry and shows the PIN field only
# when it is enforced.
ROVER_PIN = os.environ.get("ROVER_PIN", "").strip()
_PROTECTED_WS_COMMANDS = {"motor", "sensor_deploy", "pump", "camera_tilt"}


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
MAX_WS_CLIENTS = 1
pose_tracker = PoseTracker(WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV)
saved_map_origin = DB.map_origin()
_map_origin_cache = {"value": saved_map_origin}
if saved_map_origin:
    pose_tracker.set_map_origin(saved_map_origin[0], saved_map_origin[1])


def _adopt_rover_position_as_origin():
  """Make the rover's own position the map origin, with no operator input.

  Nothing needs a typed coordinate: binding the frame to the first GPS fix is
  what keeps the map, the trail and GPS fusion meaningful; before this the
  operator had to tap "Origin here".  Only runs while the rover is stopped,
  and only once.
  """
  if _map_origin_cache["value"]:
    return False
  snap = get_gps().get()
  if not snap.get("fix") or snap.get("lat") is None or snap.get("lon") is None:
    return False
  if motor_status != "STOPPED":
    return False
  try:
    apply_navigation_origin(float(snap["lat"]), float(snap["lon"]), source="rover")
  except (TypeError, ValueError, RuntimeError) as exc:
    print(f"⚠️ Could not adopt the rover position as the map origin: {exc}")
    return False
  print("📍 Rover position adopted as the map origin — no coordinates needed")
  return True

def navigation_loop():
  """Update local pose at 20 Hz (nothing is persisted)."""
  print("🧭 Navigation loop started.")
  while True:
    try:
      _adopt_rover_position_as_origin()
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

class WSClientWrapper:
    def __init__(self, ws):
        self.ws = ws
        self.lock = threading.Lock()

    def send(self, data):
        with self.lock:
            self.ws.send(data)

def handle_motor(speed, direction):
    global motor_status
    # A manual drive command is an operator override: end any sweep first so
    # the auto thread cannot fight the new command.
    if auto_controller.is_active():
        auto_controller.stop("manual drive override")
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

def handle_auto_mode(action=None, params=None, supplied_pin=None):
    """Start/stop/query the autonomous sweep (the thread lives in auto_controller).

    Starting needs the PIN when ROVER_PIN is set; stopping never does, so the
    operator can always abort a mission.
    """
    action = (action or "status").strip().lower()
    if action == "start":
        if not _pin_ok(supplied_pin):
            return {"success": False, "error": "PIN required"}
        return auto_controller.start(params)
    if action == "stop":
        stopped = auto_controller.stop("operator stop")
        return {"success": True, "stopped": stopped, **auto_controller.get_status()}
    if action == "toggle":
        if auto_controller.is_active():
            auto_controller.stop("operator stop")
            return {"success": True, "stopped": True, **auto_controller.get_status()}
        if not _pin_ok(supplied_pin):
            return {"success": False, "error": "PIN required"}
        return auto_controller.start(params)
    if action == "status":
        return {"success": True, **auto_controller.get_status()}
    return {"success": False, "error": f"unknown auto action '{action}'"}


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
    payload["soil_diagnostics"] = MOISTURE_SENSOR.getDiagnostics()
    payload["pump_active"] = WATER_PUMP.isPumpOn()
    payload["sensor_deployed"] = sensor_servo_deployed or (sensor_servo.angle is not None and sensor_servo.angle > -50)
    payload["sensor_angle"] = sensor_servo.angle
    payload["distance_cm"] = get_distance_cm()
    payload["ultrasonic_enabled"] = is_ultrasonic_enabled()
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
    payload["map_origin"] = _map_origin_cache["value"]
    payload["gps"] = get_gps().get()
    payload["memory"] = read_meminfo()
    payload["pin_required"] = bool(ROVER_PIN)
    payload["auto"] = auto_controller.get_status()
    payload["success"] = True
    return payload


def apply_navigation_origin(latitude, longitude, source="manual"):
    """Reset local pose so this lat/lon becomes map centre (0, 0 m)."""
    global _map_origin_cache
    if not (-85.0 <= latitude <= 85.0 and -180.0 <= longitude <= 180.0):
        raise ValueError("Map origin is outside supported coordinates")
    if motor_status != "STOPPED":
        raise RuntimeError("Stop the rover before changing its map start")
    _map_origin_cache["value"] = DB.save_map_origin(latitude, longitude)
    pose_tracker.set_map_origin(latitude, longitude)
    heading_rad = 0.0
    snap = get_gps().get()
    if gps_heading_usable(snap):
        # GPS course is a compass bearing; the pose frame is ENU maths angles.
        heading_rad = course_to_heading_rad(snap["course_deg"])
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


def emergency_stop_rover(reason="user disconnected"):
    global motor_status, sensor_servo_deployed
    print(f"🛑 [SAFETY FAILSAFE] Stopping all rover actions ({reason})...")
    # 0. End an autonomous sweep (unless configured to survive a disconnect)
    try:
        if auto_controller.is_active() and auto_controller.AUTO_STOP_ON_DISCONNECT:
            auto_controller.stop(f"failsafe: {reason}")
    except Exception as e:
        print(f"⚠️ Failsafe auto mode stop error: {e}")

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

    # 3. Retract sensor arm to safe home position (-90 deg)
    try:
        sensor_servo.angle = -90
        sensor_servo_deployed = False
    except Exception as e:
        print(f"⚠️ Failsafe servo retract error: {e}")

# Pi Zero W: telemetry reports how much of the shared 512 MB is left.
MEMINFO_PATH = "/proc/meminfo"


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
            time.sleep(0.2)
            with clients_lock:
                if not connected_clients:
                    continue
            broadcast_telemetry()
        except Exception as e:
            time.sleep(0.2)

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

        if not getattr(app, 'ws_broadcast_started', False):
            ws_thread = threading.Thread(target=ws_telemetry_broadcast_loop, daemon=True)
            ws_thread.start()
            app.ws_broadcast_started = True


@app.after_request
def set_static_cache_headers(response):
    """Cache static assets for a day: over the rover's flaky Wi-Fi the
    dashboard was re-downloading css/js on every single page load."""
    if request.path.startswith("/static/"):
        # The dashboard's own css/js change with every deploy, and a browser
        # holding a day-old index.js keeps showing removed controls.
        # Revalidate those two every load;
        # the vendor bundles never change, so they keep the long cache.
        if request.path.endswith(("/index.js", "/index.css")):
            response.headers["Cache-Control"] = "no-cache"
            response.headers.pop("Expires", None)
        else:
            # Overwrite (not setdefault): Flask's default is `no-cache`, which
            # would otherwise win and keep the vendor files re-downloading.
            response.headers["Cache-Control"] = "public, max-age=86400"
    return response

# WebSocket Endpoint
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
                    response["message"] = "PIN required — enter it in the header"
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
                elif cmd_type == "camera_tilt":
                    angle = handle_camera_tilt(data.get("direction"))
                    response["current_angle"] = angle
                elif cmd_type == "ultrasonic":
                    res = handle_ultrasonic(data.get("action"))
                    response.update(res)
                elif cmd_type == "auto_mode":
                    res = handle_auto_mode(
                        data.get("action"), data.get("params"), data.get("pin")
                    )
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

        # Manual driving must stop the moment its dashboard goes away.
        emergency_stop_rover("user disconnected from WebSocket")

        # Broadcast stopped telemetry to any remaining clients
        if remaining > 0:
            broadcast_telemetry()

# ----------------- Existing HTTP Endpoints (Preserved) -----------------

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
def control_auto_mode():
    """Start/stop the sweep.  JSON body: {"action": ..., "params": {...}}
    (flat parameter keys next to "action" are accepted too)."""
    payload = request.get_json(silent=True) or {}
    action = request.args.get('action') or payload.get('action')
    params = payload.get('params')
    if params is None and payload:
        params = {
            key: payload[key]
            for key in auto_controller.DEFAULT_PARAMS
            if key in payload
        }
    result = handle_auto_mode(action, params or None)
    broadcast_telemetry()
    if result.get("error") == "PIN required":
        return jsonify(result), 401
    if not result.get("success"):
        return jsonify(result), 400
    return jsonify(result), 200


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


@app.route('/api/imu/calibrate', methods=['POST'])
def calibrate_imu_endpoint():
    if motor_status != "STOPPED":
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
