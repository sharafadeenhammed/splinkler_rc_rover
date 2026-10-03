from helpers import MOISTURE_SENSOR
# from gpiozero import LED
# from gpiozero import Button
# from constants import PINOUT
from signal import pause


# moistureSensor = Button(PINOUT.PIN18)
# soilMoistureState = "Dry"

# from time import sleep

# def handleWhenPressed():
#       # soilMoistureState = "Wet"
#       print("button pressed, and soil moisture ")
  
# def handleWhenReleased():
#       # soilMoistureState = "Dry"
#       print("button released, and soil moisture ")


print("running test...")

# moistureSensor.when_pressed = handleWhenPressed
# moistureSensor.when_released = handleWhenReleased

# import time
# from helpers.encoder_controller import get_encoder_ticks

# while True:
#     print(get_encoder_ticks())
#     time.sleep(0.3)

import time
from helpers.gps_controller import gps
while True:
    print(gps.get()); time.sleep(1)

# pause()