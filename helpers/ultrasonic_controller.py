from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from constants import PINOUT

pigpio_factory = PiGPIOFactory()

# Initialize Ultrasonic Sensor (TRIG: PIN31 / GPIO 6, ECHO: PIN32 / GPIO 12)
try:
    distance_sensor = DistanceSensor(
        trigger=PINOUT.PIN31,
        echo=PINOUT.PIN32,
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

current_distance_cm = 0.0

def get_distance_cm():
    global current_distance_cm
    try:
        dist_m = distance_sensor.distance
        current_distance_cm = round(dist_m * 100.0, 1)
        return current_distance_cm
    except Exception as e:
        print(f"⚠️ Ultrasonic read error: {e}")
        return current_distance_cm

__all__ = ["distance_sensor", "get_distance_cm"]
