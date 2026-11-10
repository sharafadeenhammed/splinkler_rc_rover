"""Standalone motor stall diagnostic — no Flask app, no rover.service.

Run on the Pi with the app stopped and the rover lifted off the ground:

    sudo systemctl stop rover
    python3 test_motor.py --side left --duty 0.7 --seconds 40

Why this exists: the app sends one motor command and pigpio latches it, so a
wheel that dies on its own is either the pin being taken away from pigpio or
something downstream of the Pi (L298N, motor, wiring, power).  This script
holds the enable pin at a fixed duty and reads the pin back once a second, so
the two cases look different:

  * mode stays OUTPUT and duty matches   -> the Pi is still driving the pin;
                                            the fault is past the GPIO.
  * mode leaves OUTPUT, or duty reads 0  -> something else on the Pi seized
                                            the enable pin (a device-tree
                                            overlay claiming the same GPIO).

It talks to pigpio directly and uses constants/pinout.py, so the pins under
test are exactly the ones the app drives.  Every step is printed with
flush=True because journald/pipe output is otherwise block-buffered.
"""

import argparse
import os
import subprocess
import sys
import time

import pigpio

from constants import PINOUT

MODE_NAMES = {
    0: "INPUT",
    1: "OUTPUT",
    2: "ALT5",
    3: "ALT4",
    4: "ALT0",
    5: "ALT1",
    6: "ALT2",
    7: "ALT3",
}

# IN1/IN2 (left) and IN3/IN4 (right) plus the L298N enable pin per side.
WHEELS = {
    "left": {"forward": PINOUT.PIN11, "reverse": PINOUT.PIN12, "enable": PINOUT.ENA},
    "right": {"forward": PINOUT.PIN13, "reverse": PINOUT.PIN15, "enable": PINOUT.ENB},
}

CONFIG_CANDIDATES = ("/boot/firmware/config.txt", "/boot/config.txt")


def log(message):
    print(message, flush=True)


def check_service_stopped(force):
    """The app owns the same pins; running both gives meaningless results."""
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "rover"],
            capture_output=True, text=True, timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return True  # no systemd here: nothing to stop
    if result.stdout.strip() == "active":
        log("The rover service is still running; it owns these pins.")
        if not force:
            log("Stop it first:  sudo systemctl stop rover")
            log("(or re-run with --force to test anyway)")
            return False
        log("--force given: continuing with the service still running.")
    return True


def check_boot_config():
    """Flag overlays that can claim an enable pin behind pigpio's back."""
    path = next((p for p in CONFIG_CANDIDATES if os.path.exists(p)), None)
    if path is None:
        log("Boot config not found; skip the overlay check.")
        return
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            lines = [line.strip() for line in handle]
    except OSError as exc:
        log(f"Could not read {path}: {exc}")
        return

    enable_gpios = {PINOUT.ENA: "left ENA", PINOUT.ENB: "right ENB"}
    warnings = []
    for line in lines:
        if not line.startswith("dtoverlay="):
            continue
        overlay = line[len("dtoverlay="):].strip()
        name, _, params = overlay.partition(",")
        name = name.strip()
        if name not in ("dht11", "w1-gpio", "w1-gpio-pullup"):
            continue
        pin = 4  # kernel default for both overlays when gpiopin is omitted
        for part in params.split(","):
            if part.strip().startswith("gpiopin="):
                try:
                    pin = int(part.split("=", 1)[1])
                except ValueError:
                    pass
        if pin in enable_gpios:
            warnings.append(
                f"{line}  ->  {name} overlay sits on BCM{pin} "
                f"({enable_gpios[pin]}); the dht11 overlay expects gpiopin=21"
            )
    for warning in warnings:
        log(f"WARNING: {warning}")
    if not warnings:
        log(f"{path}: no overlay conflicts with the enable pins.")


def _release(pi, spec):
    pi.set_PWM_dutycycle(spec["enable"], 0)
    for pin in (spec["forward"], spec["reverse"]):
        pi.write(pin, 0)


def run_side(pi, side, duty, seconds, direction):
    spec = WHEELS[side]
    enable = spec["enable"]
    target = max(1, min(255, int(round(duty * 255))))

    for pin in (spec["forward"], spec["reverse"], enable):
        pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(enable, 1000)
    pi.set_PWM_dutycycle(enable, 0)

    forward_on = direction == "forward"
    pi.write(spec["forward"], 1 if forward_on else 0)
    pi.write(spec["reverse"], 0 if forward_on else 1)
    pi.set_PWM_dutycycle(enable, target)

    log("")
    log(f"=== {side.upper()} wheel: {direction.upper()} at {duty:.2f} "
        f"(PWM {target}/255) for {seconds:.0f}s ===")
    log(f"  IN:  BCM{spec['forward']} = {1 if forward_on else 0}, "
        f"BCM{spec['reverse']} = {0 if forward_on else 1}")
    log(f"  ENA: BCM{enable} @ 1000 Hz")
    log("  Watch the wheel. Every line below is a pigpio read-back of the pin.")

    started = time.monotonic()
    first_fault_at = None
    fault_kind = None
    try:
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= seconds:
                break
            time.sleep(min(1.0, max(0.05, seconds - elapsed)))
            elapsed = time.monotonic() - started
            mode = pi.get_mode(enable)
            duty_now = pi.get_PWM_dutycycle(enable)
            mode_name = MODE_NAMES.get(mode, f"mode{mode}")

            note = ""
            kind = None
            if mode != pigpio.OUTPUT:
                note = f"  !! PIN SEIZED: expected OUTPUT, pin is {mode_name}"
                kind = f"enable pin left OUTPUT ({mode_name})"
            elif duty_now != target:
                note = f"  !! PWM LOST: read-back {duty_now}, commanded {target}"
                kind = f"duty read-back changed to {duty_now}"

            if kind and first_fault_at is None:
                first_fault_at = elapsed
                fault_kind = kind

            log(f"  {elapsed:5.1f}s  mode={mode_name:<6} duty={duty_now:3d}{note}")
    finally:
        _release(pi, spec)

    log(f"  stopped {side} wheel.")
    if first_fault_at is None:
        log("  RESULT: GPIO stayed under pigpio's control for the whole run.")
        log("          If the wheel died anyway, the fault is downstream of the Pi:")
        log("          L298N channel, motor, wiring, or supply sag. Check whether")
        log("          the heatsink is hot and whether that wheel spins freely by hand.")
    else:
        log(f"  RESULT: enable pin lost at {first_fault_at:.1f}s - {fault_kind}.")
        log("          That is a software/configuration conflict on the GPIO, not the")
        log("          motor: find what took the pin (overlay, second process).")
    return first_fault_at


def main():
    parser = argparse.ArgumentParser(
        description="Hold one wheel at a fixed duty and report pigpio read-back faults"
    )
    parser.add_argument("--side", choices=("left", "right", "both"), default="left",
                        help="wheel to test; 'both' runs left then right (default: left)")
    parser.add_argument("--duty", type=float, default=0.7,
                        help="PWM duty 0.05-1.0 (default: 0.7)")
    parser.add_argument("--seconds", type=float, default=40.0,
                        help="seconds per wheel (default: 40)")
    parser.add_argument("--dir", choices=("forward", "reverse"), default="forward",
                        help="winding to energise (default: forward)")
    parser.add_argument("--force", action="store_true",
                        help="run even if rover.service is active")
    args = parser.parse_args()

    duty = max(0.05, min(1.0, args.duty))
    seconds = max(1.0, min(300.0, args.seconds))
    sides = ("left", "right") if args.side == "both" else (args.side,)

    if not check_service_stopped(args.force):
        return 2

    pi = pigpio.pi()
    if not pi.connected:
        log("pigpiod is not reachable. Start it (sudo systemctl start pigpiod) and retry.")
        return 2

    try:
        log(f"pigpio connected (pigpio version {pi.get_pigpio_version()}).")
        check_boot_config()
        log("")
        log("SAFETY: wheels off the ground, nothing in the drivetrain. Ctrl+C stops.")
        if sys.stdin.isatty():
            try:
                input("Press ENTER to start...")
            except EOFError:
                pass

        results = {}
        try:
            for side in sides:
                results[side] = run_side(pi, side, duty, seconds, args.dir)
        except KeyboardInterrupt:
            log("\nInterrupted; pins released.")
            return 130

        if len(results) > 1:
            log("")
            log("=== summary ===")
            for side, fault_at in results.items():
                state = "no GPIO fault" if fault_at is None else f"GPIO lost at {fault_at:.1f}s"
                log(f"  {side:5s}: {state}")
        return 0
    finally:
        for spec in WHEELS.values():
            _release(pi, spec)
        pi.stop()


if __name__ == "__main__":
    raise SystemExit(main())
