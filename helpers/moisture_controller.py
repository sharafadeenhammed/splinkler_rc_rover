import time
import threading
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from constants import PINOUT
from helpers.motor_controls import forward, reverse, turnRight, turnLeft, stop
from helpers.servo_controller import sensor_servo
from helpers.io_controller import MOISTURE_SENSOR, WATER_PUMP

# Check OpenCV availability for Pi Zero W camera vision
try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
    print("📷 OpenCV loaded for Pi Zero W visual edge detection.")
except ImportError:
    OPENCV_AVAILABLE = False
    print("⚠️ OpenCV not installed. Autonomous mode will rely on Ultrasonic sensor.")

pigpio_factory = PiGPIOFactory()

# Initialize Ultrasonic Sensor (TRIG: PIN31 / GPIO 6, ECHO: PIN32 / GPIO 12)
try:
    distance_sensor = DistanceSensor(
        echo=PINOUT.PIN32,
        trigger=PINOUT.PIN31,
        max_distance=4.0,
        pin_factory=pigpio_factory
    )
except Exception as e:
    print(f"⚠️ Ultrasonic sensor init error (pigpio): {e}, trying default factory...")
    distance_sensor = DistanceSensor(
        echo=PINOUT.PIN32,
        trigger=PINOUT.PIN31,
        max_distance=4.0
    )

auto_mode_active = False
current_distance_cm = 0.0
vision_status = "CLEAR"
auto_thread = None
auto_lock = threading.Lock()

OBSTACLE_THRESHOLD_CM = 20.0   # 20 cm obstacle threshold
SOIL_CHECK_INTERVAL_SEC = 10.0  # Check soil moisture every 10s of driving
latest_jpeg_frame = None       # Global frame holder for visual edge detection

def set_latest_frame(jpeg_bytes):
    global latest_jpeg_frame
    latest_jpeg_frame = jpeg_bytes

def get_distance_cm():
    global current_distance_cm
    try:
        dist_m = distance_sensor.distance
        current_distance_cm = round(dist_m * 100.0, 1)
        return current_distance_cm
    except Exception as e:
        print(f"⚠️ Ultrasonic read error: {e}")
        return current_distance_cm

def analyze_visual_edges(jpg_bytes):
    """
    Lightweight Canny Edge & Barrier Detection optimized for Raspberry Pi Zero W (ARMv6 512MB RAM).
    Downscales frame to 160x120 and evaluates lower ground ROI edge density.
    """
    global vision_status
    if not OPENCV_AVAILABLE or not jpg_bytes:
        vision_status = "CLEAR"
        return "CLEAR", 0, 0
    try:
        np_arr = np.frombuffer(jpg_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            vision_status = "CLEAR"
            return "CLEAR", 0, 0
            
        # Downscale to 160x120 to keep Pi Zero W CPU load under 5%
        img_small = cv2.resize(img, (160, 120))
        
        # Lower half Region of Interest (the path directly ahead of rover)
        roi = img_small[60:120, :]
        
        # Fast Canny Edge Detection
        edges = cv2.Canny(roi, 50, 150)
        
        # Split into Left vs Right sectors
        left_zone = edges[:, :80]
        right_zone = edges[:, 80:]
        
        left_density = int(np.sum(left_zone > 0))
        right_density = int(np.sum(right_zone > 0))
        total_density = left_density + right_density
        
        # Visual barrier threshold
        EDGE_BARRIER_THRESHOLD = 1400
        
        if total_density > EDGE_BARRIER_THRESHOLD:
            if left_density > right_density:
                vision_status = "OBSTACLE_LEFT"
                return "OBSTACLE_LEFT", left_density, right_density
            else:
                vision_status = "OBSTACLE_RIGHT"
                return "OBSTACLE_RIGHT", left_density, right_density
                
        vision_status = "CLEAR"
        return "CLEAR", left_density, right_density
    except Exception as e:
        print(f"⚠️ Visual analysis error: {e}")
        vision_status = "CLEAR"
        return "CLEAR", 0, 0

def get_vision_status():
    return vision_status

def autonomous_routine():
    global auto_mode_active
    print("🤖 Autonomous Mode Started (Dual Ultrasonic + Pi Zero W Camera Vision).")
    last_soil_check = time.time()
    
    while auto_mode_active:
        try:
            # 1. Ultrasonic Distance Check
            dist = get_distance_cm()
            
            # 2. Pi Zero W Camera Visual Edge Check
            vis_state, left_den, right_den = analyze_visual_edges(latest_jpeg_frame)
            print(f"🤖 Auto Loop | Dist: {dist}cm | Vision: {vis_state} (L:{left_den}, R:{right_den})")
            
            # 3. Obstacle Avoidance Trigger (Ultrasonic <= 20cm OR Camera Visual Barrier)
            ultrasonic_obstacle = (dist > 0 and dist <= OBSTACLE_THRESHOLD_CM)
            visual_obstacle = (vis_state != "CLEAR")
            
            if ultrasonic_obstacle or visual_obstacle:
                cause = "Ultrasonic Distance" if ultrasonic_obstacle else f"Camera Edge Barrier ({vis_state})"
                print(f"🚨 Obstacle detected via {cause}! Executing evasive turn...")
                stop()
                time.sleep(0.2)
                
                # Reverse slightly
                reverse(0.4)
                time.sleep(0.4)
                stop()
                
                # Turn towards whichever direction has fewer visual obstacles
                if vis_state == "OBSTACLE_LEFT":
                    print("↪️ Turning RIGHT to avoid left visual barrier...")
                    turnRight(0.5)
                else:
                    print("↩️ Turning LEFT to avoid right visual barrier...")
                    turnLeft(0.5)
                    
                time.sleep(0.7)
                stop()
                time.sleep(0.2)
                continue
            
            # 4. Periodic Soil Moisture Inspection
            now = time.time()
            if now - last_soil_check >= SOIL_CHECK_INTERVAL_SEC:
                print("🌱 Stopping for periodic soil moisture check...")
                stop()
                time.sleep(0.3)
                
                # Deploy sensor arm (90 deg)
                sensor_servo.angle = 90
                time.sleep(1.2)
                
                soil_state = MOISTURE_SENSOR.getSoilMoistureState()
                print(f"🌱 Soil state: {soil_state}")
                
                if soil_state.lower() == "dry":
                    print("💧 Soil DRY -> Activating Water Pump for 3 seconds...")
                    WATER_PUMP.turnOnPump()
                    pump_start = time.time()
                    while time.time() - pump_start < 3.0 and auto_mode_active:
                        time.sleep(0.2)
                    WATER_PUMP.turnOffPump()
                else:
                    print("✅ Soil WET -> Skipping pump.")
                
                # Retract sensor arm (-90 deg)
                sensor_servo.angle = -90
                time.sleep(0.8)
                last_soil_check = time.time()
                continue
            
            # 5. Normal Drive Forward
            forward(0.4)
            time.sleep(0.3)
            
        except Exception as e:
            print(f"⚠️ Autonomous loop error: {e}")
            stop()
            time.sleep(1.0)
            
    # Cleanup
    stop()
    WATER_PUMP.turnOffPump()
    sensor_servo.angle = -90
    print("🤖 Autonomous Mode Stopped.")

def start_auto_mode():
    global auto_mode_active, auto_thread
    with auto_lock:
        if not auto_mode_active:
            auto_mode_active = True
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
    return False

def toggle_auto_mode():
    global auto_mode_active
    if auto_mode_active:
        return stop_auto_mode()
    else:
        return start_auto_mode()

def is_auto_active():
    return auto_mode_active

__all__ = [
    "get_distance_cm",
    "set_latest_frame",
    "get_vision_status",
    "start_auto_mode",
    "stop_auto_mode",
    "toggle_auto_mode",
    "is_auto_active"
]
