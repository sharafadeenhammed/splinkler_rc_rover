"""JGB37-520 dual-channel encoder support (pigpio)."""

import atexit
from threading import Lock

import pigpio
from constants import PINOUT


PULSES_PER_MOTOR_REV = 11
GEAR_RATIO = 30   # set from the gearbox label, or calibrate with a 1 m test
COUNTS_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO


class HardwareEncoder:
    """Counts rising edges of A; direction from B's last known level."""

    def __init__(self, pi, pin_a, pin_b):
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        self._lock = Lock()

        for pin in (pin_a, pin_b):
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)
            pi.set_glitch_filter(pin, 20)   # ignore noise under 20 µs

        self._b_level = pi.read(pin_b)      # one-time read, outside callbacks
        self._cb_b = pi.callback(pin_b, pigpio.EITHER_EDGE, self._b_changed)
        self._cb_a = pi.callback(pin_a, pigpio.RISING_EDGE, self._a_rising)

    def _b_changed(self, gpio, level, tick):
        if level in (0, 1):
            self._b_level = level

    def _a_rising(self, gpio, level, tick):
        with self._lock:
            if self._b_level == 1:
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
        for name in ("_cb_a", "_cb_b"):
            cb = getattr(self, name, None)
            if cb is not None:
                cb.cancel()
                setattr(self, name, None)


pi = pigpio.pi()

if not pi.connected:
    print("⚠️  pigpio daemon not running! Run 'sudo pigpiod'")
    LEFT_ENCODER = None
    RIGHT_ENCODER = None
else:
    print("⚙️  Hardware Encoders initialized.")
    LEFT_ENCODER = HardwareEncoder(pi, PINOUT.LEFT_ENCODER_A, PINOUT.LEFT_ENCODER_B)
    RIGHT_ENCODER = HardwareEncoder(pi, PINOUT.RIGHT_ENCODER_A, PINOUT.RIGHT_ENCODER_B)


def shutdown_encoders():
    """Cancel callbacks first, then close the daemon connection."""
    global LEFT_ENCODER, RIGHT_ENCODER
    for enc in (LEFT_ENCODER, RIGHT_ENCODER):
        if enc is not None:
            enc.close()
    LEFT_ENCODER = RIGHT_ENCODER = None
    if pi.connected:
        pi.stop()

atexit.register(shutdown_encoders)


def get_encoder_ticks():
    if LEFT_ENCODER and RIGHT_ENCODER:
        return LEFT_ENCODER.get_ticks(), RIGHT_ENCODER.get_ticks()
    return 0, 0


def reset_encoders():
    if LEFT_ENCODER and RIGHT_ENCODER:
        LEFT_ENCODER.reset()
        RIGHT_ENCODER.reset()


def encoder_config():
    return {
        "pulses_per_motor_rev": PULSES_PER_MOTOR_REV,
        "gear_ratio": GEAR_RATIO,
        "counts_per_output_rev": COUNTS_PER_OUTPUT_REV,
        "counting_mode": "channel_a_rising_edge",
    }