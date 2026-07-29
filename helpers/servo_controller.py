from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from constants import PINOUT

pigpioFactory = PiGPIOFactory()

camera_servo = AngularServo(
    pin=PINOUT.PIN16, 
    pin_factory=pigpioFactory, 
    initial_angle=0, 
    min_angle=-90, 
    max_angle=90, 
    # min_pulse_width=0.0005, 
    # max_pulse_width=0.0025
)

sensor_servo = AngularServo(
    pin=PINOUT.PIN37,
    pin_factory=pigpioFactory,
    initial_angle=-90,
    min_angle=-90,
    max_angle=90,
)

__all__ = ["camera_servo", "sensor_servo"]
