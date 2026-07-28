import time
import threading
from helpers.motor_controls import forward, reverse, turnRight, turnLeft, stop
from helpers.servo_controller import sensor_servo
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.pump_controller import WATER_PUMP
from helpers.vision_controller import analyze_visual_edges
from helpers.ultrasonic_controller import get_distance_cm

auto_mode_active = False
auto_thread = None
auto_lock = threading.Lock()

OBSTACLE_THRESHOLD_CM = 20.0   # 20 cm obstacle threshold
SOIL_CHECK_INTERVAL_SEC = 10.0  # Check soil moisture every 10s of driving

def autonomous_routine():
    global auto_mode_active
    print("🤖 Autonomous Mode Started (Dual Ultrasonic + Pi Zero W Camera Vision).")
    last_soil_check = time.time()
    
    while auto_mode_active:
        try:
            # 1. Ultrasonic Distance Check
            dist = get_distance_cm()
            
            # 2. Pi Zero W Camera Visual Edge Check
            vis_state, left_den, right_den = analyze_visual_edges()
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
    "start_auto_mode",
    "stop_auto_mode",
    "toggle_auto_mode",
    "is_auto_active"
]
