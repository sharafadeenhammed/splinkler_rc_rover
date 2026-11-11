from helpers import motor_controls 
import time


from time import sleep
from helpers.motor_controls import _drive_forward, stop

state = True
try:
  while state:
    motor_controls._drive_forward(1)   # call once, then leave it alone
    time.sleep(20)
    motor_controls._drive_reverse(1)   # call once, then leave it alone
    sleep(20)
    state = False
except KeyboardInterrupt:
  print('stoping motors')
  motor_controls.stop()

print("motor stopped...")
motor_controls.stop()   # call once, then leave it alone