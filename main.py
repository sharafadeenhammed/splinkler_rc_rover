import sys
import signal
import time
import threading
import subprocess
import json
from flask import Flask, Response, render_template, request, jsonify

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
    get_auto_progress, set_auto_speed, set_area,
    get_encoder_ticks, reset_encoders, encoder_config
)

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
                
    except Exception as e:
      print(f"⚠️ Sensor read error: {e}")
      # Sensor chips (like DHT22 or BME280) should not be polled faster than 1-2 seconds.
      time.sleep(2.0)

def generate_frames():
    # Use rpicam-vid with the mjpeg codec for clean streaming frame outputs
    cmd = [
        'rpicam-vid',
        '-t', '0',               # Run indefinitely
        '--codec', 'mjpeg',     # Force MJPEG frame outputs
        '--inline',             # Include frame headers inline
        '-o', '-',              # Output directly to stdout pipe
        '--width', '640',       # Balanced resolution
        '--height', '480',
        '--framerate', '15',    # Cap at 15 FPS for smooth network throughput
        '-n'                    # Suppress local desktop preview window
    ]
    
    # Spawn the process and capture stdout
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    buffer = b""
    try:
        while True:
            chunk = process.stdout.read(4096)
            if not chunk:
                # If the process terminates, check stderr logs
                stderr_data = process.stderr.read()
                if stderr_data:
                    print(f"Camera Process Error: {stderr_data.decode()}")
                break
                
            buffer += chunk
            
            # Extract distinct JPEG frames out of the stream buffer
            while b'\xff\xd8' in buffer:
                start = buffer.find(b'\xff\xd8')
                end = buffer.find(b'\xff\xd9', start)
                
                if end != -1:
                    jpg = buffer[start:end+2]
                    buffer = buffer[end+2:]
                    
                    # Feed frame to camera vision analyzer
                    set_latest_frame(jpg)

                    # Yield standard multipart frame payload
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                else:
                    break
    except GeneratorExit:
        print("Client disconnected from web/mobile stream.")
    finally:
        process.terminate()
        process.wait()

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
    payload["vision_status"] = get_vision_status()
    payload["auto_active"] = is_auto_active()
    payload["auto_progress"] = get_auto_progress()
    payload["motor_status"] = motor_status
    payload["camera_angle"] = camera_current_angle
    lt, rt = get_encoder_ticks()
    payload["encoder_left"] = lt
    payload["encoder_right"] = rt
    payload["encoder_config"] = encoder_config()
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


def handle_set_area(width_m, height_m):
    """Set coverage area dimensions in metres."""
    try:
        set_area(float(width_m), float(height_m))
        return {"success": True, "area_width_m": float(width_m), "area_height_m": float(height_m)}
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
            time.sleep(0.5)
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

    if HAS_SOCK and not getattr(app, 'ws_broadcast_started', False):
        ws_thread = threading.Thread(target=ws_telemetry_broadcast_loop, daemon=True)
        ws_thread.start()
        app.ws_broadcast_started = True

# WebSocket Endpoint
if HAS_SOCK:
    @sock.route('/ws')
    def ws_handler(ws):
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
                    elif cmd_type == "set_area":
                        res = handle_set_area(data.get("width_m", 10), data.get("height_m", 10))
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
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=--frame'
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

@app.route('/set_area', methods=['GET', 'POST'])
def set_area_endpoint():
    if request.is_json:
        width_m  = request.json.get('width_m',  10)
        height_m = request.json.get('height_m', 10)
    else:
        width_m  = request.args.get('width_m',  10)
        height_m = request.args.get('height_m', 10)
    res = handle_set_area(width_m, height_m)
    broadcast_telemetry()
    return jsonify(res), 200

if __name__ == '__main__':
    # host='0.0.0.0' exposes the server to your entire local network (Wi-Fi/LAN)
    # threaded=True allows multiple backend connections without blocking
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=True, use_reloader=False)
