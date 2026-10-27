"""Optional magnetometer over I2C for drift-free heading.

Genuine Honeywell HMC5883L chips answer at I2C address 0x1E.  Most modules
sold as "HMC5883L" nowadays carry a QMC5883P clone (the die is usually
marked HP5883) which answers at 0x2C instead, so both addresses are probed
at startup.

The compass is used only to cancel MPU6050 gyro drift.  Corrections are
computed from heading *changes* rather than absolute headings, which makes
them immune to the board's mounting rotation and to local magnetic
declination: both are constants that cancel out of the difference.  The
rotation sense (which way the board faces) is learned from the rover's first
real turn before any correction is applied.

Like the other optional helpers this module fails soft: without a working
magnetometer everything still runs, just with the gyro's usual drift.
"""

import math
import os
import threading
import time

from helpers.pose_fusion import normalize_angle_rad

# Set COMPASS_ENABLED=0 to force the compass off (parity with GPS_ENABLED).
COMPASS_ENABLED = os.environ.get("COMPASS_ENABLED", "1").strip().lower() in ("1", "true", "yes")
# Low-pass weight for the raw heading EMA (per 20 Hz navigation tick).
COMPASS_SMOOTH_ALPHA = 0.25
# Weight used when folding compass/gyro disagreement into the drift
# correction (per 20 Hz navigation tick, same style as GPS_FUSION_ALPHA).
COMPASS_FUSION_ALPHA = float(os.environ.get("COMPASS_FUSION_ALPHA", "0.08"))
# Turn (radians) that must accumulate before the rotation sense is learned.
MIN_TURN_RAD = 0.8
# Clamp so a misbehaving magnetometer can never yank the heading around
# faster than this (radians per 20 Hz tick).
MAX_STEP_RAD = 0.01
# A single-sample jump larger than this (degrees) is treated as a bus glitch
# until several consecutive readings agree on the new value.  A real pivot
# stays well below 15 deg per 20 Hz tick (75 deg/s), while the observed
# glitch was a 26 deg instant hop.
SPIKE_LIMIT_RAD = math.radians(float(os.environ.get("COMPASS_SPIKE_DEG", "15")))
# Consecutive agreeing samples needed before a large jump is accepted.
SPIKE_CONFIRM = int(os.environ.get("COMPASS_SPIKE_CONFIRM", "3"))


class Compass:
    """I2C magnetometer with gyro-drift cancellation support."""

    HMC5883L_ADDR = 0x1E
    QMC5883P_ADDR = 0x2C

    # QMC5883P registers (chip id reads 0x80 - the identification signature).
    QMC_CHIP_ID = 0x00
    QMC_DATA = 0x01
    QMC_CONTROL1 = 0x0A
    QMC_CONTROL2 = 0x0B
    QMC_CHIP_ID_VALUE = 0x80

    # HMC5883L registers.
    HMC_CONFIG_A = 0x00
    HMC_CONFIG_B = 0x01
    HMC_MODE = 0x02
    HMC_DATA = 0x03

    def __init__(self, bus_number=1):
        self.bus = None
        self.available = False
        self.chip = None
        self.heading_rad = 0.0
        self.error = None
        self._seeded = False
        self._i2c_errors = 0
        self._lock = threading.RLock()
        # Drift-correction state.
        self._prev_imu = None
        self._prev_mag = None
        self._sign = None           # learned rotation sense (+1 / -1)
        self._turned = 0.0          # accumulated |rotation| while learning
        self._agreement = 0.0       # accumulated gyro*compass step products
        self._ref_imu = None        # comparison baseline once fusion starts
        self._ref_mag = None
        # Spike-rejection state for large single-sample jumps.
        self._pending_rad = None
        self._pending_hits = 0

        if not COMPASS_ENABLED:
            self.error = "compass disabled by COMPASS_ENABLED"
            return

        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus
            self.bus = SMBus(bus_number)
            if self._init_chip():
                self.available = True
                print(f"🧭 Magnetometer initialized ({self.chip}).")
            else:
                self.bus = None
                print(f"⚠️ Magnetometer unavailable; navigation keeps gyro-only heading: {self.error}")
        except Exception as exc:
            self.bus = None
            self.error = str(exc)
            print(f"⚠️ Magnetometer unavailable; navigation keeps gyro-only heading: {exc}")

    def _init_chip(self):
        """Probe for a QMC5883P clone first, then a genuine HMC5883L."""
        try:
            if self.bus.read_byte_data(self.QMC5883P_ADDR, self.QMC_CHIP_ID) == self.QMC_CHIP_ID_VALUE:
                # Normal mode, 50 Hz output, OSR 4, downsample 2 (0x55);
                # range +/-8 G with set/reset on (0x08).
                self.bus.write_byte_data(self.QMC5883P_ADDR, self.QMC_CONTROL1, 0x55)
                self.bus.write_byte_data(self.QMC5883P_ADDR, self.QMC_CONTROL2, 0x08)
                time.sleep(0.05)
                self.chip = "QMC5883P"
                return True
        except OSError:
            pass

        try:
            self.bus.read_byte(self.HMC5883L_ADDR)  # ACK probe
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC_CONFIG_A, 0x70)  # 8 avg, 15 Hz
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC_CONFIG_B, 0x20)   # +/-1.3 Ga
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC_MODE, 0x00)        # continuous
            time.sleep(0.05)
            self.chip = "HMC5883L"
            return True
        except OSError:
            pass

        self.error = "no magnetometer found at 0x1E or 0x2C"
        return False

    @staticmethod
    def _signed16(high, low):
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    @classmethod
    def _signed16le(cls, low, high):
        return cls._signed16(high, low)

    def _read_raw(self):
        """Return (x, y, z) raw field, or None while no fresh data exists."""
        if self.chip == "QMC5883P":
            data = self.bus.read_i2c_block_data(self.QMC5883P_ADDR, self.QMC_DATA, 6)
            x = self._signed16le(data[0], data[1])
            y = self._signed16le(data[2], data[3])
            z = self._signed16le(data[4], data[5])
        else:
            data = self.bus.read_i2c_block_data(self.HMC5883L_ADDR, self.HMC_DATA, 6)
            # HMC5883L reports X, Z, Y big-endian.
            x = self._signed16(data[0], data[1])
            z = self._signed16(data[2], data[3])
            y = self._signed16(data[4], data[5])
        if x == 0 and y == 0 and z == 0:
            return None  # chip still suspended / not producing data yet
        return x, y, z

    def update(self, now=None):
        """Read the sensor and refresh the smoothed magnetic heading (20 Hz)."""
        if not self.available:
            return self.snapshot()
        try:
            raw = self._read_raw()
        except Exception as exc:
            # Motor noise can glitch I2C: keep the last heading instead of
            # raising into the navigation loop (same policy as the MPU6050).
            self._i2c_errors += 1
            if self._i2c_errors in (1, 50):
                print(f"⚠️ Magnetometer read failed ({exc}) - keeping last heading")
            return self.snapshot()
        self._i2c_errors = 0
        if raw is None:
            return self.snapshot()

        measured = math.atan2(raw[1], raw[0])
        with self._lock:
            if not self._seeded:
                self.heading_rad = measured  # avoid EMA slewing from zero
                self._seeded = True
            else:
                delta = normalize_angle_rad(measured - self.heading_rad)
                if abs(delta) <= SPIKE_LIMIT_RAD:
                    self.heading_rad = normalize_angle_rad(
                        self.heading_rad + COMPASS_SMOOTH_ALPHA * delta
                    )
                    self._pending_rad = None
                    self._pending_hits = 0
                else:
                    # Suspicious jump (glitch, or a genuine very fast turn):
                    # hold the last good heading until SPIKE_CONFIRM consecutive
                    # readings agree with each other, then adopt it.  A lone
                    # bad I2C read therefore never moves the needle.
                    if self._pending_rad is not None and abs(
                        normalize_angle_rad(measured - self._pending_rad)
                    ) <= SPIKE_LIMIT_RAD:
                        self._pending_hits += 1
                    else:
                        self._pending_rad = measured
                        self._pending_hits = 1
                    if self._pending_hits >= SPIKE_CONFIRM:
                        self.heading_rad = measured
                        self._pending_rad = None
                        self._pending_hits = 0
        return self.snapshot()

    def drift_correction(self, imu_heading_rad):
        """Return this tick's heading correction (radians) to cancel gyro
        drift, or None when nothing should be applied yet.

        Until the rotation sense is learned from a real turn only ``None``
        comes back, so a wrongly oriented board can never corrupt the
        heading.  The correction is proportional feedback on the residual
        error since the baseline (compass delta vs. gyro delta), so the
        unknown mounting rotation and magnetic declination cancel out,
        applied corrections feed back through the heading itself (no
        integrator windup), and sensor noise stays bounded instead of
        integrating into a random walk.
        """
        if not self.available or imu_heading_rad is None:
            return None
        with self._lock:
            if not self._seeded:
                return None
            imu = normalize_angle_rad(float(imu_heading_rad))
            mag = self.heading_rad

            if self._sign is None:
                # Learning phase: per-tick steps are tiny, so the +/-pi wrap
                # never bites, and only real turns carry signal.
                if self._prev_imu is None:
                    self._prev_imu, self._prev_mag = imu, mag
                    return None
                d_imu = normalize_angle_rad(imu - self._prev_imu)
                d_mag = normalize_angle_rad(mag - self._prev_mag)
                self._prev_imu, self._prev_mag = imu, mag
                self._agreement += d_imu * d_mag
                self._turned += abs(d_imu)
                if self._turned >= MIN_TURN_RAD and abs(self._agreement) > 1e-3:
                    self._sign = 1 if self._agreement > 0 else -1
                    self._ref_imu, self._ref_mag = imu, mag
                    self._prev_imu = self._prev_mag = None
                    print(f"🧭 Compass rotation sense learned (sign={self._sign:+d}); gyro drift correction active.")
                return None

            if self._ref_imu is None:
                self._ref_imu, self._ref_mag = imu, mag
                return None

            # Residual = -(gyro drift) since the baseline; mounting offset
            # and declination cancel inside the difference of the deltas.
            mag_delta = normalize_angle_rad(self._sign * (mag - self._ref_mag))
            imu_delta = normalize_angle_rad(imu - self._ref_imu)
            residual = normalize_angle_rad(mag_delta - imu_delta)
            step = COMPASS_FUSION_ALPHA * residual
            return max(-MAX_STEP_RAD, min(MAX_STEP_RAD, step))

    def reset_fusion(self):
        """Restart comparison after the heading was re-anchored (GPS origin
        set or IMU recalibration).  The learned rotation sense is kept."""
        with self._lock:
            self._prev_imu = None
            self._prev_mag = None
            self._ref_imu = None
            self._ref_mag = None

    def snapshot(self):
        with self._lock:
            return {
                "available": self.available,
                "chip": self.chip,
                "heading_deg": math.degrees(self.heading_rad),
                "heading_rad": self.heading_rad,
                "sign": self._sign,
                "fusion_active": self._sign is not None,
                "i2c_errors": self._i2c_errors,
                "error": self.error,
            }


COMPASS = Compass()


def get_compass_snapshot():
    return COMPASS.snapshot()


def update_compass():
    return COMPASS.update()


def compass_drift_correction(imu_heading_rad):
    return COMPASS.drift_correction(imu_heading_rad)


def reset_compass_fusion():
    COMPASS.reset_fusion()


__all__ = [
    "COMPASS",
    "COMPASS_ENABLED",
    "get_compass_snapshot",
    "update_compass",
    "compass_drift_correction",
    "reset_compass_fusion",
]
