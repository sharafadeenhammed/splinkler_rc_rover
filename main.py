import sys
import signal
import time
import threading
import subprocess
from flask import Flask, Response, render_template, request, jsonify
from helpers import forward, reverse, turnRight, turnLeft, stop, readHumidityAndTemp, camera_servo


app = Flask(__name__)
camera_current_angle = 0
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
      if temp or humidity: print("temp: ", temp, "  humidity: ", humidity)
            
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
    payload["success"] = True
    
  return jsonify(payload), 200



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
    camera_current_angle = max(-90, camera_current_angle - step_degrees)
  elif direction == "right":
    camera_current_angle = min(90, camera_current_angle + step_degrees)
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
    camera_servo.value = None 
    
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