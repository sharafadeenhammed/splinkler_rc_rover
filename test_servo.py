# from helpers.servo_controller import sensor_servo
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from constants import PINOUT

pigpioFactory = PiGPIOFactory()

sensor_servo = AngularServo(
    pin=PINOUT.PIN37,
    pin_factory=pigpioFactory,
    initial_angle=-90,
    min_angle=-90,
    max_angle=90,
)

while True:
  print("angle 70 deg")
  sensor_servo.angle = -20
  sleep(2)
  # sensor_servo.angle = 0
  # print("angle 0 deg")
  # sleep(2)
  sensor_servo.angle = -90
  print("angle -90 deg")
  sleep(2)
