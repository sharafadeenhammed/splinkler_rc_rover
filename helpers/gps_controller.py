import threading
import serial
import pynmea2

class GPS:
    def __init__(self, port="/dev/serial0", baud=9600):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lat = self.lon = None
        self.speed_kmh = None
        self.sats = 0
        self.fix = False
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
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
            except (pynmea2.ParseError, serial.SerialException, ValueError):
                continue

    def get(self):
        return {"fix": self.fix, "lat": self.lat, "lon": self.lon,
                "sats": self.sats, "speed_kmh": self.speed_kmh}

    def close(self):
        self._run = False
        self.ser.close()

gps = GPS()


# pip install pyserial pynmea2 