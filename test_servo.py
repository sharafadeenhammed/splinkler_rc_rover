from helpers import camera_servo
from time import sleep

while True:
  print("angle 90 deg")
  camera_servo.angle = 90
  sleep(2)
  camera_servo.angle = 0
  print("angle 0 deg")
  sleep(2)
  camera_servo.angle = -90
  print("angle -90 deg")
  sleep(2)
