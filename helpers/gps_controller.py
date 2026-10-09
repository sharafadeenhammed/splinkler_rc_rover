"""Optional UART GPS reader (NMEA GGA / RMC). Fails soft when hardware is absent."""

import os
import threading
import time

try:
    import serial
    import pynmea2
except ImportError:
    serial = None
    pynmea2 = None

GPS_PORT = os.environ.get("GPS_PORT", "/dev/serial0")
GPS_BAUD = int(os.environ.get("GPS_BAUD", "9600"))
GPS_ENABLED = os.environ.get("GPS_ENABLED", "1").strip().lower() in ("1", "true", "yes")

_instance = None
_instance_lock = threading.Lock()


class _GPSUnavailable:
    """Placeholder when GPS is disabled or serial cannot be opened."""

    available = False

    def get(self):
        return {
            "available": False,
            "fix": False,
            "lat": None,
            "lon": None,
            "sats": 0,
            "speed_kmh": None,
            "course_deg": None,
            "error": "GPS not available",
        }

    def close(self):
        pass


class GPS:
    def __init__(self, port=GPS_PORT, baud=GPS_BAUD):
        self.available = False
        self.ser = None
        self.lat = self.lon = None
        self.speed_kmh = None
        self.sats = 0
        self.fix = False
        self.course_deg = None
        self._run = False
        self._t = None
        self.error = None

        if serial is None or pynmea2 is None:
            self.error = "pyserial or pynmea2 not installed"
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.available = True
            self._run = True
            self._t = threading.Thread(target=self._loop, daemon=True)
            self._t.start()
            print(f"🛰️ GPS serial opened on {port} @ {baud}")
        except (OSError, serial.SerialException) as exc:
            self.error = str(exc)
            print(f"⚠️ GPS unavailable ({port}): {exc}")

    def _loop(self):
        while self._run and self.ser:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if not line.startswith("$"):
                    continue
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self.fix = int(msg.gps_qual or 0) > 0
                    self.sats = int(msg.num_sats or 0)
                    if self.fix:
                        self.lat, self.lon = msg.latitude, msg.longitude
                elif isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
                    self.speed_kmh = (msg.spd_over_grnd or 0) * 1.852
                    if msg.true_course is not None:
                        self.course_deg = float(msg.true_course)
            except (pynmea2.ParseError, serial.SerialException, ValueError, AttributeError, OSError):
                # Never busy-spin (e.g. the serial port died) and never let the
                # reader thread die: a dead thread would leave a stale GPS fix
                # feeding the navigation loop forever.
                time.sleep(0.2)
                continue

    def get(self):
        return {
            "available": self.available,
            "fix": bool(self.fix and self.lat is not None and self.lon is not None),
            "lat": self.lat,
            "lon": self.lon,
            "sats": self.sats,
            "speed_kmh": self.speed_kmh,
            "course_deg": self.course_deg,
            "error": self.error,
        }

    def close(self):
        self._run = False
        if self.ser:
            try:
                self.ser.close()
            except OSError:
                pass
            self.ser = None


def get_gps():
    global _instance
    if not GPS_ENABLED:
        return _GPSUnavailable()
    with _instance_lock:
        if _instance is None:
            _instance = GPS(GPS_PORT, GPS_BAUD)
        return _instance


__all__ = ["GPS", "get_gps", "GPS_PORT", "GPS_BAUD", "GPS_ENABLED"]
