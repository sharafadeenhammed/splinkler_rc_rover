"""Best-effort motor/pump cut, intended for systemd ExecStopPost.

pigpiod is a separate daemon: it keeps GPIO PWM running after the app
process dies, so an OOM kill would otherwise leave the rover driving with
no operator in control.  This script zeroes the four motor direction
channels, the two L298N enable (speed) channels and the pump pin directly
via pigpio, with no dependency on the app.

Usage:  python deploy/emergency_stop.py   (exit 0 = motors cut)
"""

import pigpio

# BCM pins from constants/pinout.py — direction (IN) pins, digital now:
#   LEFT_MOTOR_FORWARD=PIN11(17), LEFT_MOTOR_REVERSE=PIN12(18),
#   RIGHT_MOTOR_FORWARD=PIN13(27), RIGHT_MOTOR_REVERSE=PIN15(22)
MOTOR_PINS = (17, 18, 27, 22)
# L298N speed enables: ENA=PIN7 (BCM 4), ENB=PIN38 (BCM 20).
# Zeroing these alone already stops both motors.
ENABLE_PINS = (4, 20)
# WATER_PUMP = PINOUT.PIN22 (BCM 25), driven as a gpiozero LED (GPIO write).
PUMP_PIN = 25


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("emergency_stop: pigpiod not running; cannot cut motors")
        return 1
    for pin in MOTOR_PINS + ENABLE_PINS:
        try:
            pi.set_PWM_dutycycle(pin, 0)  # clear any legacy PWM state
            pi.write(pin, 0)              # direction pins are digital: force LOW
        except Exception as exc:
            print(f"emergency_stop: motor pin {pin}: {exc}")
    try:
        pi.write(PUMP_PIN, 0)
    except Exception as exc:
        print(f"emergency_stop: pump pin: {exc}")
    pi.stop()
    print("emergency_stop: motor PWM and pump pin cleared")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
