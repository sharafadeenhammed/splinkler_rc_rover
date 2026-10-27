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
    _remember(0.0, 0.0)


def _drive_forward(speed):
    """Energise the physical 'forward' winding."""
    stop()
    LEFT_MOTOR_FORWARD.value  = speed
    RIGHT_MOTOR_FORWARD.value = speed
    _remember(speed, speed)


def _drive_reverse(speed):
    """Energise the physical 'reverse' winding."""
    stop()
    LEFT_MOTOR_REVERSE.value  = speed
    RIGHT_MOTOR_REVERSE.value = speed
    _remember(-speed, -speed)


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

last_left_speed = None
last_right_speed = None

def _remember(logical_left, logical_right):
    """Record the commanded wheel duties so telemetry (dashboard drive bars)
    always shows what the wheels are being asked to do.  Values are stored in
    the same convention drive_tank uses (forward-pin sign), i.e. negated when
    MOTOR_REVERSED swaps the physical channels."""
    global last_left_speed, last_right_speed
    if MOTOR_REVERSED:
        logical_left = -logical_left
        logical_right = -logical_right
    last_left_speed = logical_left
    last_right_speed = logical_right

def drive_tank(left_speed, right_speed):
    """
    Independent tank-track style control.
    Positive speeds go forward, negative go reverse.
    """
    global last_left_speed, last_right_speed
    
    # Bound speeds to -1.0 to 1.0
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    if MOTOR_REVERSED:
        left_speed = -left_speed
        right_speed = -right_speed

    # Avoid redundant RPC calls to pigpiod if speeds haven't changed
    if left_speed == last_left_speed and right_speed == last_right_speed:
        return

    with _motor_lock:
        # Left motor
        if last_left_speed is None or (left_speed >= 0) != (last_left_speed >= 0):
            LEFT_MOTOR_FORWARD.value = 0
            LEFT_MOTOR_REVERSE.value = 0
            
        if left_speed >= 0:
            if last_left_speed is None or LEFT_MOTOR_FORWARD.value != left_speed:
                LEFT_MOTOR_FORWARD.value = left_speed
        else:
            abs_left = abs(left_speed)
            if last_left_speed is None or LEFT_MOTOR_REVERSE.value != abs_left:
                LEFT_MOTOR_REVERSE.value = abs_left

        # Right motor
        if last_right_speed is None or (right_speed >= 0) != (last_right_speed >= 0):
            RIGHT_MOTOR_FORWARD.value = 0
            RIGHT_MOTOR_REVERSE.value = 0
            
        if right_speed >= 0:
            if last_right_speed is None or RIGHT_MOTOR_FORWARD.value != right_speed:
                RIGHT_MOTOR_FORWARD.value = right_speed
        else:
            abs_right = abs(right_speed)
            if last_right_speed is None or RIGHT_MOTOR_REVERSE.value != abs_right:
                RIGHT_MOTOR_REVERSE.value = abs_right
                
    last_left_speed = left_speed
    last_right_speed = right_speed


def turnRight(speed=1.0):
    """Pivot right: left side forward, right side reverse."""
    stop()
    if MOTOR_REVERSED:
        LEFT_MOTOR_REVERSE.value  = speed
        RIGHT_MOTOR_FORWARD.value = speed
    else:
        LEFT_MOTOR_FORWARD.value  = speed
        RIGHT_MOTOR_REVERSE.value = speed
    _remember(speed, -speed)


def turnLeft(speed=1.0):
    """Pivot left: right side forward, left side reverse."""
    stop()
    if MOTOR_REVERSED:
        RIGHT_MOTOR_REVERSE.value  = speed
        LEFT_MOTOR_FORWARD.value  = speed
    else:
        LEFT_MOTOR_REVERSE.value  = speed
        RIGHT_MOTOR_FORWARD.value = speed
    _remember(-speed, speed)
