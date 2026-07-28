from gpiozero import Button
from constants import PINOUT

# Soil moisture sensor hardware controller
class MOISTURE_SENSOR:
    moistureSensor = Button(PINOUT.PIN18)
    soilMoistureState = "Dry"

    @staticmethod
    def getSoilMoistureState():
        return MOISTURE_SENSOR.soilMoistureState

    @staticmethod
    def handleWhenPressed():
        MOISTURE_SENSOR.soilMoistureState = "Wet"
        print("🌱 Soil Moisture state updated: WET")

    @staticmethod
    def handleWhenReleased():
        MOISTURE_SENSOR.soilMoistureState = "Dry"
        print("🌱 Soil Moisture state updated: DRY")

# Set hardware callbacks for moisture sensor
MOISTURE_SENSOR.moistureSensor.when_pressed = MOISTURE_SENSOR.handleWhenPressed
MOISTURE_SENSOR.moistureSensor.when_released = MOISTURE_SENSOR.handleWhenReleased

__all__ = ["MOISTURE_SENSOR"]
