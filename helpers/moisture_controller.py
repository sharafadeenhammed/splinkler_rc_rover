import os
import time

from gpiozero import Button
from constants import PINOUT

# Soil moisture sensor hardware controller
class MOISTURE_SENSOR:
    moistureSensor = Button(PINOUT.PIN18)
    # Most LM393 soil modules pull DO low when the soil crosses the wet
    # threshold. Set SOIL_SENSOR_WET_WHEN_PRESSED=0 for modules wired/configured
    # with the opposite comparator polarity.
    wet_when_pressed = os.environ.get("SOIL_SENSOR_WET_WHEN_PRESSED", "1").strip().lower() not in {
        "0", "false", "no", "off"
    }
    soilMoistureState = "Unknown"
    last_pressed = None
    last_error = ""
    last_read_at = None

    @staticmethod
    def getSoilMoistureState():
        try:
            pressed = bool(MOISTURE_SENSOR.moistureSensor.is_pressed)
            MOISTURE_SENSOR.last_pressed = pressed
            MOISTURE_SENSOR.soilMoistureState = (
                "Wet" if pressed == MOISTURE_SENSOR.wet_when_pressed else "Dry"
            )
            MOISTURE_SENSOR.last_error = ""
            MOISTURE_SENSOR.last_read_at = time.time()
        except Exception as e:
            print(f"⚠️ Error reading moisture sensor hardware state: {e}")
            MOISTURE_SENSOR.soilMoistureState = "Unknown"
            MOISTURE_SENSOR.last_error = str(e)
            MOISTURE_SENSOR.last_read_at = time.time()
        return MOISTURE_SENSOR.soilMoistureState

    @staticmethod
    def getDiagnostics():
        return {
            "state": MOISTURE_SENSOR.soilMoistureState,
            "pressed": MOISTURE_SENSOR.last_pressed,
            "wet_when_pressed": MOISTURE_SENSOR.wet_when_pressed,
            "error": MOISTURE_SENSOR.last_error,
            "read_at": MOISTURE_SENSOR.last_read_at,
        }

    @staticmethod
    def handleWhenPressed():
        MOISTURE_SENSOR.soilMoistureState = (
            "Wet" if MOISTURE_SENSOR.wet_when_pressed else "Dry"
        )

    @staticmethod
    def handleWhenReleased():
        MOISTURE_SENSOR.soilMoistureState = (
            "Dry" if MOISTURE_SENSOR.wet_when_pressed else "Wet"
        )

# Set hardware callbacks for moisture sensor
MOISTURE_SENSOR.moistureSensor.when_pressed = MOISTURE_SENSOR.handleWhenPressed
MOISTURE_SENSOR.moistureSensor.when_released = MOISTURE_SENSOR.handleWhenReleased

__all__ = ["MOISTURE_SENSOR"]
