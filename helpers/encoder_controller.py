"""JGB37-520 dual-channel encoder support.

The motors in this rover expose two Hall-effect outputs.  The seller lists
11 pulses per motor revolution; the gearbox multiplies that count at the
wheel/output shaft.  We count one edge of channel A and use channel B for
direction, which gives one count per encoder pulse (not x4 quadrature).
"""

from threading import Lock

import pigpio
from constants import PINOUT


PULSES_PER_MOTOR_REV = 11
# Change this to match the ratio printed on the gearbox.  The common 1:30
# version is retained as the project default, but this must be calibrated if
# your motor has a different gearbox.
GEAR_RATIO = 30
COUNTS_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO

class HardwareEncoder:
    """
    Reads quadrature encoder ticks using the pigpio daemon.
    This runs at the C-level, ensuring we don't miss high-speed motor ticks.
    """
    def __init__(self, pi, pin_a, pin_b):
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        self._lock = Lock()
        
        # Setup pins as inputs with pull-up resistors
        self.pi.set_mode(pin_a, pigpio.INPUT)
        self.pi.set_mode(pin_b, pigpio.INPUT)
        self.pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Attach hardware interrupt on the Rising Edge of Pin A
        self.cb = self.pi.callback(pin_a, pigpio.RISING_EDGE, self._edge_detected)
        
    def _edge_detected(self, gpio, level, tick):
        # Read Pin B to determine direction of rotation
        b_level = self.pi.read(self.pin_b)
        with self._lock:
            if b_level == 1:
                self.ticks += 1
            else:
                self.ticks -= 1
            
    def get_ticks(self):
        with self._lock:
            return self.ticks
        
    def reset(self):
        with self._lock:
            self.ticks = 0

    def close(self):
        """Release the pigpio callback when the controller is shut down."""
        if self.cb is not None:
            self.cb.cancel()
            self.cb = None

# Connect to the local pigpio daemon
pi = pigpio.pi()

# BCM GPIO Pins based on your wiring
LEFT_PIN_A = PINOUT.LEFT_ENCODER_A
LEFT_PIN_B = PINOUT.LEFT_ENCODER_B

RIGHT_PIN_A = PINOUT.RIGHT_ENCODER_A
RIGHT_PIN_B = PINOUT.RIGHT_ENCODER_B

if not pi.connected:
    print("⚠️  pigpio daemon not running! Encoders will not work. Run 'sudo pigpiod'")
    LEFT_ENCODER = None
    RIGHT_ENCODER = None
else:
    print("⚙️  Hardware Encoders initialized.")
    LEFT_ENCODER = HardwareEncoder(pi, LEFT_PIN_A, LEFT_PIN_B)
    RIGHT_ENCODER = HardwareEncoder(pi, RIGHT_PIN_A, RIGHT_PIN_B)

def get_encoder_ticks():
    """Returns a tuple of (left_ticks, right_ticks)"""
    if LEFT_ENCODER and RIGHT_ENCODER:
        print("right and left encoders handelers active");
        return LEFT_ENCODER.get_ticks(), RIGHT_ENCODER.get_ticks()
    print("left and right encoders not active.");
    return 0, 0

def reset_encoders():
    """Resets both tick counters to zero"""
    if LEFT_ENCODER and RIGHT_ENCODER:
        LEFT_ENCODER.reset()
        RIGHT_ENCODER.reset()


def encoder_config():
    """Return the active hardware calibration for telemetry and diagnostics."""
    return {
        "pulses_per_motor_rev": PULSES_PER_MOTOR_REV,
        "gear_ratio": GEAR_RATIO,
        "counts_per_output_rev": COUNTS_PER_OUTPUT_REV,
        "counting_mode": "channel_a_rising_edge",
    }
