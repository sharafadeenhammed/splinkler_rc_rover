import pigpio
import time
from constants import PINOUT

pi = pigpio.pi()
ultrasonic_enabled = True
current_distance_cm = 0.0

TRIGGER_PIN = PINOUT.PIN31
ECHO_PIN = PINOUT.PIN32

_ping_start_tick = None
_cb = None

def _echo_callback(gpio, level, tick):
    global _ping_start_tick, current_distance_cm
    if level == 1:
        _ping_start_tick = tick
    elif level == 0 and _ping_start_tick is not None:
        diff = pigpio.tickDiff(_ping_start_tick, tick)
        # Speed of sound ~34300 cm/s -> 0.0343 cm/us -> divide by 2 for round trip = 0.01715
        dist = diff * 0.01715
        current_distance_cm = round(min(dist, 400.0), 1)
        _ping_start_tick = None

def init_ultrasonic_hardware():
    global _cb
    if not pi.connected:
        print("⚠️ pigpio not connected. Ultrasonic sensor disabled.")
        return False
    if _cb is not None:
        return True
    try:
        pi.set_mode(TRIGGER_PIN, pigpio.OUTPUT)
        pi.set_mode(ECHO_PIN, pigpio.INPUT)
        pi.write(TRIGGER_PIN, 0)
        _cb = pi.callback(ECHO_PIN, pigpio.EITHER_EDGE, _echo_callback)
        print("🔊 Ultrasonic Distance Sensor initialized (pigpio manual trigger).")
        return True
    except Exception as e:
        print(f"⚠️ Ultrasonic init error: {e}")
        return False

# Initialize on startup if enabled
init_ultrasonic_hardware()

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
    if ultrasonic_enabled:
        return disable_ultrasonic()
    else:
        return enable_ultrasonic()

def is_ultrasonic_enabled():
    return ultrasonic_enabled

def get_distance_cm():
    global current_distance_cm
    if not ultrasonic_enabled or not pi.connected:
        return None
    try:
        # Trigger a 10 microsecond pulse
        pi.gpio_trigger(TRIGGER_PIN, 10, 1)
        # The callback will update current_distance_cm asynchronously.
        return current_distance_cm
    except Exception as e:
        print(f"⚠️ Ultrasonic read error: {e}")
        return current_distance_cm

__all__ = [
    "get_distance_cm",
    "enable_ultrasonic",
    "disable_ultrasonic",
    "toggle_ultrasonic",
    "is_ultrasonic_enabled"
]
