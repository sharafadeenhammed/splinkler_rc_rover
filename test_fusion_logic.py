"""Hardware-free tests for the compass drift-correction fusion and the pure
pose_fusion maths (runs on the dev box and on the Pi):

    python test_fusion_logic.py

Modules are loaded by file path so importing helpers/__init__ (gpiozero,
pigpio, ...) is never triggered on a machine without the rover's stack.
"""
import importlib.util
import math
import os
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))


def load_helper(name):
    if "helpers" not in sys.modules:
        pkg = types.ModuleType("helpers")
        pkg.__path__ = [os.path.join(HERE, "helpers")]
        sys.modules["helpers"] = pkg
    full = f"helpers.{name}"
    if full not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            full, os.path.join(HERE, "helpers", f"{name}.py")
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[full] = module
        spec.loader.exec_module(module)
    return sys.modules[full]


def make_compass(compass_cls):
    """Compass instance with the I2C hardware stripped away."""
    c = compass_cls.__new__(compass_cls)
    c.available = True
    c.chip = "SIM"
    c.heading_rad = 0.0
    c.error = None
    c._seeded = True
    c._i2c_errors = 0
    c._lock = threading.RLock()
    c._prev_imu = None
    c._prev_mag = None
    c._sign = None
    c._turned = 0.0
    c._agreement = 0.0
    c._ref_imu = None
    c._ref_mag = None
    return c


def _simulate(c, sign, mount_offset, drift_rate, ticks=600, turn_ticks=60):
    """Persistent MPU heading: raw gyro integrates (true delta + drift), then
    the returned correction step is applied — exactly main.py's order."""
    true = 0.0
    prev_true = 0.0
    imu = 0.0
    learned_at = None
    max_step = 0.0
    for t in range(ticks):
        if t < turn_ticks:
            true += 0.02  # Phase A: turning
        d_true = true - prev_true
        prev_true = true
        imu += d_true + drift_rate  # raw gyro integration
        c.heading_rad = sign * true + mount_offset
        corr = c.drift_correction(imu)
        if corr is not None:
            imu += corr
            max_step = max(max_step, abs(corr))
        if c._sign is not None and learned_at is None:
            learned_at = t
    return true, imu, learned_at, max_step


def test_compass_drift_cancellation():
    hmc = load_helper("hmc5883l")
    beta = hmc.COMPASS_FUSION_ALPHA
    scenarios = [
        (+1, math.radians(37), 0.0005),
        (-1, math.radians(-120), -0.0005),
        (+1, 0.0, -0.0003),
        (-1, math.radians(89), math.radians(0.05) / 20),  # realistic gyro
    ]
    for sign, offset, drift in scenarios:
        c = make_compass(hmc.Compass)
        true, imu, learned_at, _ = _simulate(c, sign, offset, drift)
        residual = math.degrees(imu - true)
        # Closed form: drift before the baseline is unknowable; afterwards the
        # proportional loop settles at a lag of drift/beta.
        pre = math.degrees(drift * learned_at)
        lag = math.degrees(abs(drift) / beta) * (1 if drift > 0 else -1)
        post = math.degrees(drift * (600 - 1 - learned_at))
        efficiency = 100 * (1 - (residual - pre) / post) if post else 100
        assert c._sign == sign, f"learned sign {c._sign} != {sign}"
        assert abs(residual - (pre + lag)) < 0.25, (sign, offset, drift, residual)
        assert efficiency > 90, f"compensation only {efficiency:.1f}%"
    print("PASS compass drift cancellation (both senses, both drift signs, "
          "model matches closed form, >90% compensation)")


def test_compass_withholds_until_turned():
    hmc = load_helper("hmc5883l")
    c = make_compass(hmc.Compass)
    imu = 0.0
    for _ in range(400):
        imu += 0.0005
        c.heading_rad = 0.0
        assert c.drift_correction(imu) is None, "must not correct before a turn"
    print("PASS compass withholds correction until rotation sense is learned")


def test_compass_reset_keeps_sign():
    hmc = load_helper("hmc5883l")
    c = make_compass(hmc.Compass)
    c._sign = -1
    c._ref_imu = 1.0
    c._ref_mag = 2.0
    c._prev_imu = 3.0
    c.reset_fusion()
    assert c._sign == -1, "learned sign must survive a GPS-origin reset"
    assert c._ref_imu is None and c._prev_imu is None
    print("PASS reset_fusion keeps the learned sign, clears baselines")


def test_pose_fusion_maths():
    pf = load_helper("pose_fusion")

    # geo <-> local round trip
    x, y = pf.geo_to_local(6.5244, 3.3792, 6.5240, 3.3790)
    lat, lon = pf.local_to_geo(x, y, 6.5240, 3.3790)
    assert abs(lat - 6.5244) < 1e-9 and abs(lon - 3.3792) < 1e-9, (lat, lon)

    # angle normalisation and lerp across the wrap (±π are the same angle)
    assert abs(abs(pf.normalize_angle_rad(3 * math.pi)) - math.pi) < 1e-9
    lerped = pf.angle_lerp(math.radians(179), math.radians(-179), 0.5)
    assert abs(math.degrees(lerped)) > 179 or abs(math.degrees(lerped)) < -179, lerped

    # GPS quality gates
    assert not pf.gps_fix_usable(None)
    assert not pf.gps_fix_usable({"available": True, "fix": True, "lat": 1.0, "lon": 2.0, "sats": 3})
    good = {"available": True, "fix": True, "lat": 1.0, "lon": 2.0, "sats": 8}
    assert pf.gps_fix_usable(good)
    assert not pf.gps_heading_usable(good)  # no course / too slow
    moving = dict(good, course_deg=90.0, speed_kmh=5.0)
    assert pf.gps_heading_usable(moving)
    print("PASS pose_fusion maths (geo round trip, wrap lerp, GPS gates)")


if __name__ == "__main__":
    test_compass_drift_cancellation()
    test_compass_withholds_until_turned()
    test_compass_reset_keeps_sign()
    test_pose_fusion_maths()
    print("ALL FUSION LOGIC TESTS PASSED")
