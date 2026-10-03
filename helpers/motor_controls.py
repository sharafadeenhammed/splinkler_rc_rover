from gpiozero import LED, Button, Motor, PWMOutputDevice
from time import sleep
from threading import Lock
from constants import PINOUT
from gpiozero.pins.pigpio import PiGPIOFactory


# initialize pin factory
factory = PiGPIOFactory()
_motor_lock = Lock()

# left motor pin init
LEFT_MOTOR_FORWARD  = PWMOutputDevice(PINOUT.PIN11, initial_value=0, frequency=1000, pin_factory=factory)
LEFT_MOTOR_REVERSE  = PWMOutputDevice(PINOUT.PIN12, initial_value=0, frequency=1000, pin_factory=factory)

# right motor pin init
RIGHT_MOTOR_FORWARD = PWMOutputDevice(PINOUT.PIN13, initial_value=0, frequency=1000, pin_factory=factory)
RIGHT_MOTOR_REVERSE = PWMOutputDevice(PINOUT.PIN15, initial_value=0, frequency=1000, pin_factory=factory)

# ─── Polarity correction ──────────────────────────────────────────────────────
# Set True if the rover physically moves BACKWARDS when forward() is called.
# This swaps the forward/reverse channels in software so you don't have to
# re-wire the H-bridge.
MOTOR_REVERSED = False
# ─────────────────────────────────────────────────────────────────────────────


def stop():
    with _motor_lock:
        LEFT_MOTOR_FORWARD.value  = 0
        LEFT_MOTOR_REVERSE.value  = 0
        RIGHT_MOTOR_FORWARD.value = 0
        RIGHT_MOTOR_REVERSE.value = 0


def _drive_forward(speed):
    """Energise the physical 'forward' winding."""
    stop()
    LEFT_MOTOR_FORWARD.value  = speed
    RIGHT_MOTOR_FORWARD.value = speed


def _drive_reverse(speed):
    """Energise the physical 'reverse' winding."""
    stop()
    LEFT_MOTOR_REVERSE.value  = speed
    RIGHT_MOTOR_REVERSE.value = speed


def forward(speed=1.0):
    """Move rover in the logical FORWARD direction."""
    if MOTOR_REVERSED:
        _drive_reverse(speed)
    else:
        _drive_forward(speed)


def reverse(speed=1.0):
    """Move rover in the logical REVERSE direction."""
    if MOTOR_REVERSED:
        _drive_forward(speed)
    else:
        _drive_reverse(speed)

def drive_tank(left_speed, right_speed):
    """
    Independent tank-track style control.
    Positive speeds go forward, negative go reverse.
    """
    # Bound speeds to -1.0 to 1.0
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    if MOTOR_REVERSED:
        left_speed = -left_speed
        right_speed = -right_speed

    # Clear both legs before selecting a direction.  This avoids briefly
    # driving both H-bridge inputs during a forward/reverse transition.
    with _motor_lock:
        LEFT_MOTOR_FORWARD.value = 0
        LEFT_MOTOR_REVERSE.value = 0
        RIGHT_MOTOR_FORWARD.value = 0
        RIGHT_MOTOR_REVERSE.value = 0

        if left_speed >= 0:
            LEFT_MOTOR_FORWARD.value = left_speed
        else:
            LEFT_MOTOR_REVERSE.value = abs(left_speed)

        if right_speed >= 0:
            RIGHT_MOTOR_FORWARD.value = right_speed
        else:
            RIGHT_MOTOR_REVERSE.value = abs(right_speed)


def turnRight(speed=1.0):
    """Pivot right: left side forward, right side reverse."""
    stop()
    if MOTOR_REVERSED:
        LEFT_MOTOR_REVERSE.value  = speed
        RIGHT_MOTOR_FORWARD.value = speed
    else:
        LEFT_MOTOR_FORWARD.value  = speed
        RIGHT_MOTOR_REVERSE.value = speed


def turnLeft(speed=1.0):
    """Pivot left: right side forward, left side reverse."""
    stop()
    if MOTOR_REVERSED:
        RIGHT_MOTOR_REVERSE.value = speed
        LEFT_MOTOR_FORWARD.value  = speed
    else:
        LEFT_MOTOR_REVERSE.value  = speed
        RIGHT_MOTOR_FORWARD.value = speed
