from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from constants import PINOUT

pigpio_factory = PiGPIOFactory()
distance_sensor = None
ultrasonic_enabled = True

def init_ultrasonic_hardware():
    global distance_sensor
    if distance_sensor is not None:
        return distance_sensor
    try:
        distance_sensor = DistanceSensor(
            echo=PINOUT.PIN32,
            trigger=PINOUT.PIN31,
            max_distance=4.0,
            pin_factory=pigpio_factory
        )
        print("🔊 Ultrasonic Distance Sensor initialized (pigpio).")
    except Exception as e:
        print(f"⚠️ Ultrasonic init (pigpio) error: {e}. Trying default factory...")
        try:
            distance_sensor = DistanceSensor(
                echo=PINOUT.PIN32,
                trigger=PINOUT.PIN31,
                max_distance=4.0
            )
            print("🔊 Ultrasonic Distance Sensor initialized (default).")
        except Exception as err:
            print(f"⚠️ Ultrasonic hardware not detected: {err}")
            distance_sensor = None
    return distance_sensor

# Initialize on startup if enabled
init_ultrasonic_hardware()

current_distance_cm = 0.0

def enable_ultrasonic():
    global ultrasonic_enabled
    ultrasonic_enabled = True
    init_ultrasonic_hardware()
    print("🔊 Ultrasonic Sensor ENABLED in code.")
    return True

def disable_ultrasonic():
    global ultrasonic_enabled
    ultrasonic_enabled = False
    print("🔇 Ultrasonic Sensor DISABLED in code.")
    return False

def toggle_ultrasonic():
    global ultrasonic_enabled
    if ultrasonic_enabled:
        return disable_ultrasonic()
    else:
        return enable_ultrasonic()

def is_ultrasonic_enabled():
    return ultrasonic_enabled

def get_distance_cm():
    global current_distance_cm
    if not ultrasonic_enabled or distance_sensor is None:
        return None
    try:
        dist_m = distance_sensor.distance
        current_distance_cm = round(dist_m * 100.0, 1)
        return current_distance_cm
    except Exception as e:
        print(f"⚠️ Ultrasonic read error: {e}")
        return current_distance_cm

__all__ = [
    "distance_sensor",
    "get_distance_cm",
    "enable_ultrasonic",
    "disable_ultrasonic",
    "toggle_ultrasonic",
    "is_ultrasonic_enabled"
]
