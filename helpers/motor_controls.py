from gpiozero import LED, Button, Motor, PWMOutputDevice
from time import sleep
from constants import PINOUT
from gpiozero.pins.pigpio import PiGPIOFactory


# initialize pin factory
factory = PiGPIOFactory()

# left motor pin init
LEFT_MOTOR_FORWARD = PWMOutputDevice(PINOUT.PIN11, initial_value=0, frequency=1000, pin_factory = factory)
LEFT_MOTOR_REVERSE = PWMOutputDevice(PINOUT.PIN12, initial_value=0, frequency=1000, pin_factory = factory)

# right motor pin init
RIGHT_MOTOR_FORWARD = PWMOutputDevice(PINOUT.PIN13, initial_value=0, frequency=1000, pin_factory = factory)
RIGHT_MOTOR_REVERSE = PWMOutputDevice(PINOUT.PIN15, initial_value=0, frequency=1000, pin_factory = factory)

def blink_led(led_pin):
  led = LED(led_pin)
  led.on()
  print("led on")
  sleep(1)
  led.off()
  print("led off")
  sleep(1)
  
def stop():
  LEFT_MOTOR_FORWARD.value = 0
  LEFT_MOTOR_REVERSE.value = 0
  RIGHT_MOTOR_FORWARD.value = 0
  RIGHT_MOTOR_REVERSE.value = 0
  sleep(0.1)
  
def forward(speed):
  stop()
  LEFT_MOTOR_FORWARD.value = speed
  RIGHT_MOTOR_FORWARD.value = speed

def reverse(speed):
  stop()
  LEFT_MOTOR_REVERSE.value = speed
  RIGHT_MOTOR_REVERSE.value = speed

def turnRight(speed):
  stop()
  LEFT_MOTOR_FORWARD.value = speed
  RIGHT_MOTOR_REVERSE.value = speed

def turnLeft(speed):
  stop()
  LEFT_MOTOR_REVERSE.value = speed
  RIGHT_MOTOR_FORWARD.value = speed
  
  
  