import time
import threading
from helpers.motor_controls import forward, reverse, turnRight, turnLeft, stop
from helpers.servo_controller import sensor_servo
from helpers.moisture_controller import MOISTURE_SENSOR
from helpers.pump_controller import WATER_PUMP
from helpers.vision_controller import analyze_visual_edges
from helpers.ultrasonic_controller import get_distance_cm

auto_mode_active = False
use_ultrasonic_in_auto = True  # Toggle flag to enable/disable ultrasonic sensor in auto navigation
auto_thread = None
auto_lock = threading.Lock()

OBSTACLE_THRESHOLD_CM = 20.0   # 20 cm obstacle threshold
SOIL_CHECK_INTERVAL_SEC = 10.0  # Check soil moisture every 10s of driving
WATERING_DURATION_SEC = 3.0    # Duration (seconds) to pump water when dry soil detected
AUTO_DRIVE_SPEED = 0.7         # Cruising speed (0.0 to 1.0) during autonomous patrol

def enable_ultrasonic_in_auto():
    global use_ultrasonic_in_auto
    use_ultrasonic_in_auto = True
    print("🔊 Ultrasonic sensor ENABLED for Autonomous Navigation.")
    return True

def disable_ultrasonic_in_auto():
    global use_ultrasonic_in_auto
    use_ultrasonic_in_auto = False
    print("🔇 Ultrasonic sensor DISABLED for Autonomous Navigation.")
    return False

def toggle_ultrasonic_in_auto():
    global use_ultrasonic_in_auto
    if use_ultrasonic_in_auto:
        return disable_ultrasonic_in_auto()
    else:
        return enable_ultrasonic_in_auto()

def is_ultrasonic_enabled_in_auto():
    return use_ultrasonic_in_auto

def autonomous_routine():
    global auto_mode_active
    mode_desc = "Ultrasonic + Vision" if use_ultrasonic_in_auto else "Camera Vision Only"
    print(f"🤖 Autonomous Mode Started ({mode_desc}).")
    last_soil_check = time.time()
    
    while auto_mode_active:
        try:
            # 1. Ultrasonic Distance Check (only if enabled in code)
            dist = get_distance_cm() if use_ultrasonic_in_auto else None
            
            # 2. Pi Zero W Camera Visual Edge Check
            vis_state, left_den, right_den = analyze_visual_edges()
            dist_str = f"{dist}cm" if (use_ultrasonic_in_auto and dist is not None) else "DISABLED"
            print(f"🤖 Auto Loop | Dist: {dist_str} | Vision: {vis_state} (L:{left_den}, R:{right_den})")
            
            # 3. Obstacle Avoidance Trigger
            ultrasonic_obstacle = (use_ultrasonic_in_auto and dist is not None and dist > 0 and dist <= OBSTACLE_THRESHOLD_CM)
            visual_obstacle = (vis_state != "CLEAR")
            
            if ultrasonic_obstacle or visual_obstacle:
                cause = "Ultrasonic Distance" if ultrasonic_obstacle else f"Camera Edge Barrier ({vis_state})"
                print(f"🚨 Obstacle detected via {cause}! Executing evasive turn...")
                stop()
                time.sleep(0.2)
                
                # Reverse slightly
                reverse(AUTO_DRIVE_SPEED)
                time.sleep(0.4)
                stop()
                
                # Turn towards whichever direction has fewer visual obstacles
                if vis_state == "OBSTACLE_LEFT":
                    print("↪️ Turning RIGHT to avoid left visual barrier...")
                    turnRight(AUTO_DRIVE_SPEED)
                else:
                    print("↩️ Turning LEFT to avoid right visual barrier...")
                    turnLeft(AUTO_DRIVE_SPEED)
                    
                time.sleep(0.7)
                stop()
                time.sleep(0.2)
                continue
            
            # 4. Periodic Soil Moisture Inspection
            now = time.time()
            if now - last_soil_check >= SOIL_CHECK_INTERVAL_SEC:
                print("🌱 Rover halting to check soil moisture...")
                stop()
                WATER_PUMP.turnOffPump()
                time.sleep(0.5)
                
                # Deploy sensor probe into the ground (-20 deg)
                print("🌱 Deploying moisture probe into soil...")
                sensor_servo.angle = -20
                time.sleep(1.2)  # Settle time in dirt
                
                soil_state = MOISTURE_SENSOR.getSoilMoistureState()
                print(f"🌱 Soil moisture status: {soil_state}")
                
                if str(soil_state).strip().lower() == "dry":
                    print(f"💧 DRY ground detected! Running water pump for {WATERING_DURATION_SEC}s...")
                    WATER_PUMP.turnOnPump()
                    pump_start = time.time()
                    while (time.time() - pump_start < WATERING_DURATION_SEC) and auto_mode_active:
                        time.sleep(0.1)
                    WATER_PUMP.turnOffPump()
                    print("💧 Ground watered successfully.")
                    time.sleep(0.5)
                else:
                    print("✅ Soil is WET / moist. Skipping pump -> continuing search for dry ground.")
                    WATER_PUMP.turnOffPump()
                
                # Safely retract sensor probe back up (-90 deg) before moving
                print("🌱 Retracting moisture sensor arm...")
                sensor_servo.angle = -90
                time.sleep(0.8)
                last_soil_check = time.time()
                continue
            
            # 5. Normal Drive Forward: keep searching for dry ground
            forward(AUTO_DRIVE_SPEED)
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
        sensor_servo.angle = -90
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
    "is_auto_active",
    "enable_ultrasonic_in_auto",
    "disable_ultrasonic_in_auto",
    "toggle_ultrasonic_in_auto",
    "is_ultrasonic_enabled_in_auto"
]
