from gpiozero import Button, LED
from constants import PINOUT


# soil moisture sensor class
class MOISTURE_SENSOR:
  moistureSensor = Button(PINOUT.PIN18)
  soilMoistureState = "Dry"
  
  @staticmethod
  def getSoilMoistureState():
    return MOISTURE_SENSOR.soilMoistureState
  
  @staticmethod
  def handleWhenPressed():
      MOISTURE_SENSOR.soilMoistureState = "Wet"
      print("button pressed, and soil moisture is ", MOISTURE_SENSOR.soilMoistureState)
  
  @staticmethod
  def handleWhenReleased():
      MOISTURE_SENSOR.soilMoistureState = "Dry"
      print("button released, and soil moisture is", MOISTURE_SENSOR.soilMoistureState)


print("running test...")

# set callback on moisture sensor 
MOISTURE_SENSOR.moistureSensor.when_pressed = MOISTURE_SENSOR.handleWhenPressed
MOISTURE_SENSOR.moistureSensor.when_released  = MOISTURE_SENSOR.handleWhenReleased

# pump class
class WATER_PUMP:
  pump = LED(PINOUT.PIN22)
  is_active = False

  @classmethod
  def turnOnPump(cls):
    cls.pump.on()
    cls.is_active = True
    print("💧 Water Pump TURNED ON")
    return True

  @classmethod
  def turnOffPump(cls):
    cls.pump.off()
    cls.is_active = False
    print("⏹️ Water Pump TURNED OFF")
    return False

  @classmethod
  def togglePump(cls):
    if cls.is_active:
      return cls.turnOffPump()
    else:
      return cls.turnOnPump()

  @classmethod
  def isPumpOn(cls):
    return cls.is_active


__all__ = ["MOISTURE_SENSOR", "WATER_PUMP"]