"""One-off calibration: measure clear-scene and synthetic-obstacle edge
densities under BOTH backends, on real frames from the rover's camera."""
import io
import os
import subprocess
import sys

FRAMES = []
for i in range(3):
    out = subprocess.run(
        ["rpicam-still", "-n", "-o", "-", "--width", "160", "--height", "120", "-q", "85"],
        capture_output=True, timeout=30,
    )
    if out.returncode != 0 or not out.stdout:
        print("CAPTURE FAILED:", out.stderr.decode()[:200])
        sys.exit(1)
    FRAMES.append(out.stdout)
    print(f"captured clear frame {i+1}: {len(out.stdout)} bytes")

import numpy as np
from PIL import Image

# Synthetic "obstacle" frame: dense high-contrast texture grafted onto the
# lower half of a real clear frame (validates the detector's obstacle regime).
base = Image.open(io.BytesIO(FRAMES[0])).convert("L").resize((160, 120), Image.BILINEAR)
arr = np.asarray(base).copy()
rng = np.random.default_rng(7)
arr[60:120, :] = rng.integers(0, 255, size=(60, 160), dtype=np.uint8)
buf = io.BytesIO()
Image.fromarray(arr).save(buf, format="JPEG", quality=85)
OBSTACLE = buf.getvalue()
print(f"synthetic obstacle frame: {len(OBSTACLE)} bytes")

def pil_stats(jpg, edge_t):
    img = Image.open(io.BytesIO(jpg)).convert("L").resize((160, 120), Image.BILINEAR)
    a = np.asarray(img, dtype=np.float32)
    roi = a[60:120, :]
    gy, gx = np.gradient(roi)
    mag = np.hypot(gx, gy)
    l = int((mag[:, :80] > edge_t).sum())
    r = int((mag[:, 80:] > edge_t).sum())
    return l, r, l + r, float(mag.max())

def cv2_stats(jpg):
    import cv2
    a = np.frombuffer(jpg, np.uint8)
    img = cv2.imdecode(a, cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, (160, 120))
    roi = small[60:120, :]
    edges = cv2.Canny(roi, 50, 150)
    l = int((edges[:, :80] > 0).sum())
    r = int((edges[:, 80:] > 0).sum())
    return l, r, l + r

print("\n=== gradient magnitude ranges (clear frames) ===")
for i, f in enumerate(FRAMES):
    _, _, _, mx = pil_stats(f, 30)
    print(f"clear{i}: max_grad={mx:.1f}")

print("\n=== PIL backend densities across edge thresholds ===")
print(f"{'frame':<18}{'e=20':>18}{'e=30':>18}{'e=40':>18}{'e=60':>18}")
for label, f in [("clear0", FRAMES[0]), ("clear1", FRAMES[1]), ("clear2", FRAMES[2]), ("SYN_OBSTACLE", OBSTACLE)]:
    row = f"{label:<18}"
    for et in (20, 30, 40, 60):
        l, r, t, _ = pil_stats(f, et)
        row += f"{t:>18}"
    print(row)

print("\n=== CV2 (Canny) densities for scale comparison ===")
for label, f in [("clear0", FRAMES[0]), ("clear1", FRAMES[1]), ("clear2", FRAMES[2]), ("SYN_OBSTACLE", OBSTACLE)]:
    l, r, t = cv2_stats(f)
    print(f"{label:<18} cv2 total={t:>6}  (clear-limit is 1400)")
