# Camera visual edge detection controller optimized for Raspberry Pi Zero W

import io
import os
import threading

# Two detector backends:
#   pil (default): Pillow + numpy gradient detection.  Imports in well under a
#                  second and costs ~15 MB resident — affordable for the
#                  always-on obstruction loop.  The old OpenCV stack was not:
#                  cv2 drags libgdal/libvtk and ~70 MB of shared objects onto
#                  a 512 MB board (measured: the app sat at 125 MB RSS).
#   cv2:           the original Canny implementation, kept as a fallback and
#                  selectable with VISION_BACKEND=cv2.
# If the configured backend fails to import, the other one is tried
# automatically so vision never dies over a missing package.
VISION_BACKEND = os.environ.get("VISION_BACKEND", "pil").strip().lower()

# pil-backend tuning: the per-pixel gradient cut-off, and the count of edge
# pixels in the lower (ground) half that means "visual barrier ahead".
# Calibrated against the cv2 baseline on the rover's own camera; both can be
# overridden in the environment without a code change.
VISION_EDGE_THRESHOLD = float(os.environ.get("VISION_EDGE_THRESHOLD", "30"))
VISION_DENSITY_THRESHOLD = int(os.environ.get("VISION_DENSITY_THRESHOLD", "1400"))

# Shared "chosen backend is importable" flag.  The historical name is kept
# because auto_controller and main import around it.
OPENCV_AVAILABLE = False
cv2 = None
np = None
Image = None
VISION_BACKEND_ACTIVE = None

_cv_load_done = threading.Event()
_cv_load_lock = threading.Lock()
_cv_loader_started = False


def _load_vision_deps():
    global cv2, np, Image, OPENCV_AVAILABLE, VISION_BACKEND_ACTIVE
    order = ["cv2", "pil"] if VISION_BACKEND == "cv2" else ["pil", "cv2"]
    errors = []
    for backend in order:
        try:
            if backend == "cv2":
                import cv2 as _cv2
                import numpy as _np
                with _cv_load_lock:
                    cv2 = _cv2
                    np = _np
            else:
                import numpy as _np
                from PIL import Image as _Image
                with _cv_load_lock:
                    np = _np
                    Image = _Image
            with _cv_load_lock:
                VISION_BACKEND_ACTIVE = backend
                OPENCV_AVAILABLE = True
            print(f"📷 Vision backend loaded: {backend}")
            return
        except Exception as exc:  # ImportError, or broken native libs
            errors.append(f"{backend}: {exc}")
    with _cv_load_lock:
        OPENCV_AVAILABLE = False
    print(f"⚠️ Vision backends unavailable ({'; '.join(errors)}). Vision analysis disabled.")


def ensure_opencv_loaded(timeout=15.0):
    """Start the background backend import (once) and wait for it (bounded).

    Called before the autonomous routine starts moving so obstacle vision is
    ready while the rover is still stationary.  Returns True when the backend
    is usable.  Safe to call repeatedly: after the loader finishes this is a
    no-op, whether the load succeeded or not.
    """
    global _cv_loader_started
    with _cv_load_lock:
        if not _cv_loader_started:
            _cv_loader_started = True
            threading.Thread(target=_load_vision_deps, daemon=True, name="vision-loader").start()
    _cv_load_done.wait(timeout=timeout)
    return OPENCV_AVAILABLE

vision_status = "CLEAR"
latest_jpeg_frame = None

def set_latest_frame(jpeg_bytes):
    global latest_jpeg_frame
    latest_jpeg_frame = jpeg_bytes

def get_latest_frame():
    return latest_jpeg_frame

last_analyzed_frame_id = None
last_analyzed_result = ("CLEAR", 0, 0)


def _analyze_pil(jpg_bytes):
    """Gradient edge density on a 160x120 downscale (memory-light path)."""
    img = Image.open(io.BytesIO(jpg_bytes)).convert("L").resize((160, 120), Image.BILINEAR)
    img_small = np.asarray(img, dtype=np.float32)

    # Lower half Region of Interest (the path directly ahead of rover)
    roi = img_small[60:120, :]
    grad_y, grad_x = np.gradient(roi)
    magnitude = np.hypot(grad_x, grad_y)

    # Split into Left vs Right sectors
    left_density = int((magnitude[:, :80] > VISION_EDGE_THRESHOLD).sum())
    right_density = int((magnitude[:, 80:] > VISION_EDGE_THRESHOLD).sum())
    total_density = left_density + right_density

    if total_density > VISION_DENSITY_THRESHOLD:
        status = "OBSTACLE_LEFT" if left_density > right_density else "OBSTACLE_RIGHT"
    else:
        status = "CLEAR"
    return status, left_density, right_density


def _analyze_cv2(jpg_bytes):
    """Original Canny path (kept verbatim as the fallback backend)."""
    np_arr = np.frombuffer(jpg_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return "CLEAR", 0, 0

    # Downscale to 160x120 to keep Pi Zero W CPU load under 5%
    img_small = cv2.resize(img, (160, 120))

    # Lower half Region of Interest (the path directly ahead of rover)
    roi = img_small[60:120, :]

    # Fast Canny Edge Detection
    edges = cv2.Canny(roi, 50, 150)

    # Split into Left vs Right sectors
    left_zone = edges[:, :80]
    right_zone = edges[:, 80:]

    left_density = int(np.sum(left_zone > 0))
    right_density = int(np.sum(right_zone > 0))
    total_density = left_density + right_density

    # Visual barrier threshold
    if total_density > VISION_DENSITY_THRESHOLD:
        status = "OBSTACLE_LEFT" if left_density > right_density else "OBSTACLE_RIGHT"
    else:
        status = "CLEAR"
    return status, left_density, right_density


def analyze_visual_edges(jpg_bytes=None):
    """
    Lightweight edge & barrier detection optimized for Raspberry Pi Zero W.
    Downscales the frame to 160x120 and evaluates lower ground ROI edge
    density, then classifies CLEAR / OBSTACLE_LEFT / OBSTACLE_RIGHT.
    """
    global vision_status, last_analyzed_frame_id, last_analyzed_result

    frame_to_process = jpg_bytes if jpg_bytes is not None else latest_jpeg_frame

    # No camera frame -> nothing to analyse, so never pay for a backend here.
    # This is the common case while driving manually.
    if not frame_to_process:
        vision_status = "CLEAR"
        return "CLEAR", 0, 0

    # A real analysis is due: make sure the backend is loaded (bounded wait
    # for the background import instead of blind-reporting CLEAR while it runs).
    if not OPENCV_AVAILABLE:
        ensure_opencv_loaded(timeout=5.0)
    if not OPENCV_AVAILABLE:
        vision_status = "CLEAR"
        return "CLEAR", 0, 0

    frame_id = id(frame_to_process)
    if frame_id == last_analyzed_frame_id:
        vision_status = last_analyzed_result[0]
        return last_analyzed_result

    try:
        if VISION_BACKEND_ACTIVE == "cv2":
            status, left_density, right_density = _analyze_cv2(frame_to_process)
        else:
            status, left_density, right_density = _analyze_pil(frame_to_process)

        vision_status = status
        last_analyzed_frame_id = frame_id
        last_analyzed_result = (status, left_density, right_density)
        return last_analyzed_result

    except Exception as e:
        print(f"⚠️ Visual analysis error: {e}")
        vision_status = "CLEAR"
        return "CLEAR", 0, 0

def get_vision_status():
    return vision_status

__all__ = [
    "set_latest_frame",
    "get_latest_frame",
    "analyze_visual_edges",
    "get_vision_status",
    "ensure_opencv_loaded",
]
