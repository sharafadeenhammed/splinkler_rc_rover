"""Optional MPU6050 attitude/heading reader over I2C.

The MPU6050 has no magnetometer, so heading is relative to the heading at
startup/calibration and will slowly drift.  It is still useful for correcting
short straight runs when combined with wheel encoders.

Noise handling (measured on the rover, motors off):
  * gyro-Z carries a warm-up bias of ~0.27 deg/s, which walked the heading
    ~16 deg/min while parked - the "80 deg to 54 deg while stationary" bug.
  * the raw rate has ~0.14 deg/s of white noise and pitch/roll jitter of
    ~0.7 deg peak-to-peak.
So the rate is low-passed before integration, a still gate freezes the
heading (and slowly re-learns the bias) whenever the rover is parked, and
pitch/roll get a display-side EMA.
"""

import math
import os
import threading
import time

# Still gate: |gyro-Z rate| below this (deg/s) counts as stationary.  Sized
# from the measured bias+noise envelope (0.27 +/- 0.14 dps, peaks to 0.7).
STILL_DPS = float(os.environ.get("IMU_STILL_DPS", "1.0"))
# Consecutive 20 Hz ticks inside the deadband before the gate engages
# (10 ticks = 0.5 s), so a slow nudge is never mistaken for parking.
STILL_TICKS = int(os.environ.get("IMU_STILL_TICKS", "10"))
# Adaptive bias tracking while gated: cancels the warm-up/thermal drift with a
# ~5 s time constant at 20 Hz instead of letting it integrate into heading.
BIAS_ALPHA = float(os.environ.get("IMU_BIAS_ALPHA", "0.01"))
# Low-pass weight for the integrated gyro rate (kills single-sample spikes).
RATE_ALPHA = 0.25
# Low-pass weight for the displayed pitch/roll attitude.
ATT_ALPHA = 0.15


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
        self._i2c_errors = 0
        self._lock = threading.RLock()
        # Filter state.
        self._rate_f = 0.0        # low-passed gyro rate actually integrated
        self._still_ticks = 0     # consecutive ticks inside the still band

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
        try:
            accel, gyro = self._raw()
        except Exception as exc:
            # Transient I2C glitches are common while the motors run: keep the
            # last good attitude instead of raising into the navigation loop.
            self._i2c_errors += 1
            if self._i2c_errors in (1, 50):
                print(f"⚠️ MPU6050 I2C read failed ({exc}) - reusing last attitude")
            return self.snapshot()
        self._i2c_errors = 0
        with self._lock:
            rate = gyro[2] - self.gyro_bias_z
            if abs(rate) <= STILL_DPS:
                self._still_ticks += 1
            else:
                self._still_ticks = 0

            if self._still_ticks >= STILL_TICKS:
                # Parked: freeze the heading so noise cannot random-walk it,
                # and slowly drag the bias to the current reading so the
                # warm-up drift is gone by the next time the rover moves.
                self.gyro_bias_z += BIAS_ALPHA * (gyro[2] - self.gyro_bias_z)
                self._rate_f = 0.0
                self.gyro_z_dps = 0.0
            else:
                self._rate_f = RATE_ALPHA * rate + (1.0 - RATE_ALPHA) * self._rate_f
                self.gyro_z_dps = self._rate_f
                if self.last_update is not None:
                    dt = min(0.1, max(0.0, now - self.last_update))
                    self.heading_rad = (self.heading_rad + math.radians(self._rate_f) * dt + math.pi) % (2 * math.pi) - math.pi

            self.last_update = now
            # Attitude for the display: EMA smooths the ~0.7 deg jitter.
            roll = math.degrees(math.atan2(accel[1], accel[2]))
            pitch = math.degrees(math.atan2(-accel[0], math.hypot(accel[1], accel[2])))
            self.roll_deg = ATT_ALPHA * roll + (1.0 - ATT_ALPHA) * self.roll_deg
            self.pitch_deg = ATT_ALPHA * pitch + (1.0 - ATT_ALPHA) * self.pitch_deg
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


def apply_heading_correction(correction_rad):
    """Nudge the integrated heading by an externally computed correction
    (e.g. compass-estimated gyro drift) without touching the update clock."""
    with MPU._lock:
        MPU.heading_rad = (MPU.heading_rad + float(correction_rad) + math.pi) % (2 * math.pi) - math.pi


__all__ = [
    "MPU",
    "get_imu_snapshot",
    "update_imu",
    "calibrate_imu",
    "reset_imu_heading",
    "set_imu_heading_rad",
    "apply_heading_correction",
]
