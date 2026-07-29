import sys
import signal
import time
import threading
import subprocess
from flask import Flask, Response, render_template, request, jsonify
from helpers import (
    forward, reverse, turnRight, turnLeft, stop, 
    readHumidityAndTemp, camera_servo, sensor_servo, 
    MOISTURE_SENSOR, WATER_PUMP,
    get_distance_cm, set_latest_frame, get_vision_status,
    enable_ultrasonic, disable_ultrasonic, toggle_ultrasonic, is_ultrasonic_enabled,
    start_auto_mode, stop_auto_mode, toggle_auto_mode, is_auto_active
)

app = Flask(__name__)
camera_current_angle = 0
sensor_servo_deployed = False
sensor_data = {
  "temperature": 0.0,
  "humidity": 0.0,
  "soil":"dry",
  "last_updated": "Never"
}
data_lock = threading.Lock()

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

# mount background task on server start
@app.before_request
def initialize_sensor_thread():
  if not getattr(app, 'sensor_thread_started', False):
    bg_thread = threading.Thread(target=read_sensor_loop, daemon=True)
    bg_thread.start()
    app.sensor_thread_started = True

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
        # Safety: Disengage auto mode if user manually overrides motor controls
        if is_auto_active():
            stop_auto_mode()

        speed_percent = int(request.args.get('speed', 0))
        direction = request.args.get('dir', 'forward') # 'forward' or 'backward'
        
        speed_percent = max(0, min(100, speed_percent))
        duty_cycle = speed_percent / 100.0
        
        if speed_percent == 0:
            # Full stop: cut power to both sides of the H-bridge
            stop()
            status = "STOPPED"
        elif direction == 'forward':
            forward(duty_cycle)       # Ensure opposite side is grounded first!
            status = f"FORWARD ({speed_percent}%)"
        elif direction == 'reverse':
            reverse(duty_cycle) # Apply speed to backward channel
            status = f"REVERSE ({speed_percent}%)"
        elif direction == 'right':
            turnRight(duty_cycle) # Apply speed to right channel
            status = f"TURNING RIGHT ({speed_percent}%)"
        elif direction == 'left':
            turnLeft(duty_cycle) # Apply speed to left channel
            status = f"TURNING LEFT ({speed_percent}%)"
        elif direction == 'stop':
            stop() # stop
            status = "STOPPED"
            
        print(status)
        return jsonify({"success": True, "status": status})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

@app.route("/sensor_readings", methods=['GET'])
def getHumidityReading():
  payload = {}
  with data_lock:
    payload = sensor_data.copy()
    payload["soil"] = MOISTURE_SENSOR.soilMoistureState
    payload["pump_active"] = WATER_PUMP.isPumpOn()
    payload["sensor_deployed"] = sensor_servo_deployed
    payload["distance_cm"] = get_distance_cm()
    payload["ultrasonic_enabled"] = is_ultrasonic_enabled()
    payload["vision_status"] = get_vision_status()
    payload["auto_active"] = is_auto_active()
    payload["success"] = True
    
  return jsonify(payload), 200


@app.route('/ultrasonic', methods=['GET', 'POST'])
def control_ultrasonic():
  action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
  
  if action == "enable":
    enabled = enable_ultrasonic()
  elif action == "disable":
    enabled = disable_ultrasonic()
  elif action == "toggle" or not action:
    enabled = toggle_ultrasonic()
  elif action == "status":
    enabled = is_ultrasonic_enabled()
  else:
    return jsonify({"success": False, "message": f"Invalid action '{action}'."}), 400

  return jsonify({
    "success": True,
    "ultrasonic_enabled": enabled,
    "status": "ENABLED 🔊" if enabled else "DISABLED 🔇"
  }), 200




@app.route('/auto_mode', methods=['GET', 'POST'])
def control_auto_mode():
  action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
  
  if action == "start":
    active = start_auto_mode()
  elif action == "stop":
    active = stop_auto_mode()
  elif action == "toggle" or not action:
    active = toggle_auto_mode()
  elif action == "status":
    active = is_auto_active()
  else:
    return jsonify({"success": False, "message": f"Invalid action '{action}'."}), 400

  return jsonify({
    "success": True,
    "auto_active": active,
    "status": "AUTONOMOUS 🤖" if active else "MANUAL 🎮"
  }), 200



@app.route('/deploy_sensor', methods=['GET', 'POST'])
def deploy_sensor():
  global sensor_servo_deployed
  action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
  
  if action == "deploy" or (action == "toggle" and not sensor_servo_deployed):
    sensor_servo.angle = -20
    sensor_servo_deployed = True
  elif action == "retract" or (action == "toggle" and sensor_servo_deployed):
    sensor_servo.angle = -90
    sensor_servo_deployed = False
  elif not action:
    # Toggle default
    sensor_servo_deployed = not sensor_servo_deployed
    sensor_servo.angle = -20 if sensor_servo_deployed else -90
  else:
    return jsonify({"success": False, "message": f"Invalid action '{action}'. Use 'deploy', 'retract', or 'toggle'."}), 400

  time.sleep(0.1)
  return jsonify({
    "success": True,
    "is_deployed": sensor_servo_deployed,
    "angle": sensor_servo.angle,
    "status": "DEPLOYED (90°)" if sensor_servo_deployed else "RETRACTED (-90°)"
  }), 200


@app.route('/water_pump', methods=['GET', 'POST'])
def control_pump():
  action = request.args.get('action') or (request.json.get('action') if request.is_json else None)
  
  if action == "on":
    is_active = WATER_PUMP.turnOnPump()
  elif action == "off":
    is_active = WATER_PUMP.turnOffPump()
  elif action == "toggle" or not action:
    is_active = WATER_PUMP.togglePump()
  elif action == "status":
    is_active = WATER_PUMP.isPumpOn()
  else:
    return jsonify({"success": False, "message": f"Invalid action '{action}'."}), 400

  return jsonify({
    "success": True,
    "pump_active": is_active,
    "status": "RUNNING 💧" if is_active else "OFF ⏹️"
  }), 200


@app.route('/camera_tilt', methods=['GET'])
def tilt_camera():
  global camera_current_angle
  direction = request.args.get('direction')
  step_degrees = 15  # Adjust this value to make the panning faster or slower
  if not direction:
    return jsonify({
      "status": "error", 
      "message": "Missing 'direction' query parameter."
    }), 400
  if direction == "left":
    camera_current_angle = max(-90, camera_current_angle + step_degrees)
  elif direction == "right":
    camera_current_angle = min(90, camera_current_angle - step_degrees)
  elif direction == "center":
    camera_current_angle = 0
  else:
    return jsonify({
      "status": "error", 
      "message": f"Invalid direction '{direction}'. Use 'left', 'right', or 'center'."
    }), 400
  try:
    camera_servo.angle = camera_current_angle
    time.sleep(0.1)
    # camera_servo.value = None 
    
    
  except Exception as e:
    return jsonify({
      "status": "error",
      "message": f"Hardware error: {str(e)}"
    }), 500

  return jsonify({
    "status": "success", 
    "current_angle": camera_current_angle
  })


if __name__ == '__main__':
    # host='0.0.0.0' exposes the server to your entire local network (Wi-Fi/LAN)
    # threaded=True allows multiple backend connections without blocking
    # signal.signal(signal.SIGINT, handle_exit)
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=True, use_reloader=False)