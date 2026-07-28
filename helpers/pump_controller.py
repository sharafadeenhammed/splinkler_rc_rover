from gpiozero import LED
from constants import PINOUT

# Water pump hardware controller
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

__all__ = ["WATER_PUMP"]
