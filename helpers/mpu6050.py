"""Optional MPU6050 attitude/heading reader over I2C.

The MPU6050 has no magnetometer, so heading is relative to the heading at
startup/calibration and will slowly drift.  It is still useful for correcting
short straight runs when combined with wheel encoders.
"""

import math
import threading
import time


class MPU6050:
    ADDRESS = 0x68
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B

    def __init__(self, bus_number=1, address=ADDRESS):
        self.bus = None
        self.address = address
        self.available = False
        self.heading_rad = 0.0
        self.gyro_bias_z = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.gyro_z_dps = 0.0
        self.last_update = None
        self._lock = threading.RLock()

        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus
            self.bus = SMBus(bus_number)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 9)  # 100 Hz
            self.bus.write_byte_data(self.address, self.CONFIG, 0x03)
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # ±250°/s
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)  # ±2g
            self.available = True
            print("⚙️ MPU6050 initialized at 0x%02X." % self.address)
        except Exception as exc:
            print(f"⚠️ MPU6050 unavailable; encoder-only navigation will be used: {exc}")

    @staticmethod
    def _signed(high, low):
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def _raw(self):
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)
        accel = [self._signed(data[i], data[i + 1]) / 16384.0 for i in (0, 2, 4)]
        gyro = [self._signed(data[i], data[i + 1]) / 131.0 for i in (8, 10, 12)]
        return accel, gyro

    def calibrate(self, samples=200, delay=0.005):
        """Measure stationary gyro bias. Keep the rover still while calling."""
        if not self.available:
            return False
        total = 0.0
        for _ in range(samples):
            _, gyro = self._raw()
            total += gyro[2]
            time.sleep(delay)
        self.gyro_bias_z = total / samples
        return True

    def update(self, now=None):
        if not self.available:
            return self.snapshot()
        now = now or time.monotonic()
        accel, gyro = self._raw()
        with self._lock:
            if self.last_update is not None:
                dt = min(0.1, max(0.0, now - self.last_update))
                self.heading_rad = (self.heading_rad + math.radians(gyro[2] - self.gyro_bias_z) * dt + math.pi) % (2 * math.pi) - math.pi
            self.last_update = now
            self.gyro_z_dps = gyro[2] - self.gyro_bias_z
            self.roll_deg = math.degrees(math.atan2(accel[1], accel[2]))
            self.pitch_deg = math.degrees(math.atan2(-accel[0], math.hypot(accel[1], accel[2])))
        return self.snapshot()

    def reset_heading(self):
        with self._lock:
            self.heading_rad = 0.0
            self.last_update = time.monotonic()

    def snapshot(self):
        with self._lock:
            return {
                "available": self.available,
                "heading_deg": math.degrees(self.heading_rad),
                "heading_rad": self.heading_rad,
                "pitch_deg": self.pitch_deg,
                "roll_deg": self.roll_deg,
                "gyro_z_dps": self.gyro_z_dps,
            }


MPU = MPU6050()


def get_imu_snapshot():
    return MPU.snapshot()


def update_imu():
    return MPU.update()


def calibrate_imu(samples=200):
    return MPU.calibrate(samples=samples)


def reset_imu_heading():
    MPU.reset_heading()


def set_imu_heading_rad(heading_rad):
    """Align integrated gyro heading with an external reference (e.g. GPS course)."""
    with MPU._lock:
        MPU.heading_rad = (float(heading_rad) + math.pi) % (2 * math.pi) - math.pi
        MPU.last_update = time.monotonic()


__all__ = [
    "MPU",
    "get_imu_snapshot",
    "update_imu",
    "calibrate_imu",
    "reset_imu_heading",
    "set_imu_heading_rad",
]
