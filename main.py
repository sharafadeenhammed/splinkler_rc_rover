import os
import sys
import signal
import time
import threading
import subprocess
import json
import select
import math
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
    get_distance_cm, set_latest_frame, get_vision_status,
    enable_ultrasonic, disable_ultrasonic, toggle_ultrasonic, is_ultrasonic_enabled,
    start_auto_mode, stop_auto_mode, toggle_auto_mode, is_auto_active,
    get_auto_progress, set_auto_speed,
    set_area_polygon,
    get_encoder_ticks, reset_encoders, encoder_config
)
from helpers.mpu6050 import get_imu_snapshot, update_imu, calibrate_imu, reset_imu_heading
from helpers.navigation_database import DB, PoseTracker
from helpers.auto_controller import WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV

app = Flask(__name__)
sock = Sock(app) if HAS_SOCK else None

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
pose_tracker = PoseTracker(DB, WHEEL_DIAMETER_M, WHEEL_TRACK_M, TICKS_PER_WHEEL_REV)
current_run_id = None
saved_area = DB.active_area()
saved_map_origin = DB.map_origin()
_map_origin_cache = {"value": saved_map_origin}
if saved_area and len(saved_area.get("polygon", [])) >= 3:
    try:
        set_area_polygon(saved_area["polygon"])
    except (TypeError, ValueError):
        pass

# Cache for the active work area so telemetry doesn't hit the database every build.
_area_cache = {"value": None, "ts": 0.0}


def cached_active_area():
    now = time.time()
    if now - _area_cache["ts"] > 5.0:
        _area_cache["value"] = DB.active_area()
        _area_cache["ts"] = now
    return _area_cache["value"]


def navigation_loop():
  """Update local pose and persist navigation telemetry at 20 Hz."""
  global current_run_id
  print("🧭 Navigation loop started.")
  while True:
    try:
      imu = update_imu()
      left_ticks, right_ticks = get_encoder_ticks()
      pose_tracker.update(left_ticks, right_ticks, imu)
      if not is_auto_active() and pose_tracker.run_id:
        DB.finish_run(pose_tracker.run_id, "stopped")
        pose_tracker.run_id = None
        current_run_id = None
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

    def subscribe(self):
        with self.lock:
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

    def _start_locked(self):
        command = [
            "rpicam-vid", "-t", "0", "--codec", "mjpeg", "--quality", "25",
            "--inline", "-o", "-", "--width", "160", "--height", "120",
            "--framerate", "4", "-n",
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
        last_vision = 0.0
        try:
            while process.poll() is None:
                with self.lock:
                    idle = self.clients == 0 and time.monotonic() - self.last_client_at > 5.0
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
                    now = time.monotonic()
                    if now - last_vision >= 1.0:
                        set_latest_frame(frame)
                        last_vision = now
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
    global current_run_id
    if action == "start":
        active = start_auto_mode()
        if active and current_run_id is None:
            area = DB.active_area()
            current_run_id = DB.start_run(area["id"] if area else None)
            pose_tracker.run_id = current_run_id
    elif action == "stop":
        active = stop_auto_mode()
        if current_run_id is not None:
            DB.finish_run(current_run_id, "stopped")
            current_run_id = None
            pose_tracker.run_id = None
    elif action == "toggle" or not action:
        active = toggle_auto_mode()
        if active and current_run_id is None:
            area = DB.active_area()
            current_run_id = DB.start_run(area["id"] if area else None)
            pose_tracker.run_id = current_run_id
        elif not active and current_run_id is not None:
            DB.finish_run(current_run_id, "stopped")
            current_run_id = None
            pose_tracker.run_id = None
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
    payload["vision_status"] = get_vision_status()
    payload["auto_active"] = is_auto_active()
    payload["auto_progress"] = get_auto_progress()
    payload["motor_status"] = motor_status
    payload["camera_angle"] = camera_current_angle
    lt, rt = get_encoder_ticks()
    payload["encoder_left"] = lt
    payload["encoder_right"] = rt
    payload["encoder_config"] = encoder_config()
    payload["imu"] = get_imu_snapshot()
    payload["pose"] = pose_tracker.snapshot()
    payload["active_area"] = cached_active_area()
    payload["map_origin"] = _map_origin_cache["value"]
    payload["success"] = True
    return payload


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

# Mount background tasks on server start
@app.before_request
def initialize_background_threads():
    if not getattr(app, 'sensor_thread_started', False):
        bg_thread = threading.Thread(target=read_sensor_loop, daemon=True)
        bg_thread.start()
        app.sensor_thread_started = True

    if not getattr(app, 'navigation_thread_started', False):
        nav_thread = threading.Thread(target=navigation_loop, daemon=True)
        nav_thread.start()
        app.navigation_thread_started = True

    if HAS_SOCK and not getattr(app, 'ws_broadcast_started', False):
        ws_thread = threading.Thread(target=ws_telemetry_broadcast_loop, daemon=True)
        ws_thread.start()
        app.ws_broadcast_started = True

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
        CLIENT_HEARTBEAT_TIMEOUT_SEC = 6.0  # Watchdog: disconnect silent dead sockets

        try:
            while True:
                msg = ws.receive(timeout=0.2)
                now = time.time()

                if msg is None:
                    # Check for dead socket timeout
                    if now - last_activity > CLIENT_HEARTBEAT_TIMEOUT_SEC:
                        print("⚠️ WebSocket client heartbeat timeout. Disconnecting...")
                        break
                    continue

                last_activity = now

                try:
                    data = json.loads(msg) if isinstance(msg, str) else json.loads(msg.decode('utf-8'))
                    cmd_type = data.get("type")
                    response = {"type": "response", "success": True, "command": cmd_type}

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
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_auto_mode(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/deploy_sensor', methods=['GET', 'POST'])
def deploy_sensor():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_sensor_deploy(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/water_pump', methods=['GET', 'POST'])
def control_pump():
    action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
    res = handle_water_pump(action)
    broadcast_telemetry()
    return jsonify({"success": True, **res}), 200

@app.route('/camera_tilt', methods=['GET'])
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
    if is_auto_active() or motor_status != "STOPPED":
        return jsonify({"success": False, "error": "Stop the rover before changing its map start"}), 409
    payload = request.get_json(silent=True) or {}
    try:
        latitude = float(payload["latitude"])
        longitude = float(payload["longitude"])
        if not (-85.0 <= latitude <= 85.0 and -180.0 <= longitude <= 180.0):
            raise ValueError("Map centre is outside supported coordinates")
        _map_origin_cache["value"] = DB.save_map_origin(latitude, longitude)
        pose_tracker.reset(0.0, 0.0, 0.0)
        reset_imu_heading()
        return jsonify({"success": True, "map_origin": [latitude, longitude]}), 200
    except (KeyError, TypeError, ValueError) as exc:
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
    return jsonify({"success": ok, "imu": get_imu_snapshot()}), (200 if ok else 503)


@app.route('/api/telemetry', methods=['GET'])
def navigation_history():
    run_id = request.args.get('run_id', type=int)
    limit = request.args.get('limit', default=500, type=int)
    return jsonify({"success": True, "telemetry": DB.recent_telemetry(run_id, limit)}), 200


@app.route('/api/runs', methods=['GET'])
def navigation_runs():
    return jsonify({"success": True, "runs": DB.recent_runs()}), 200

if __name__ == '__main__':
    # host='0.0.0.0' exposes the server to your entire local network (Wi-Fi/LAN)
    # threaded=True allows multiple backend connections without blocking
    # debug=False: debug mode adds overhead and exposes the Werkzeug debugger to the network
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False, use_reloader=False)
