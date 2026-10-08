# Camera visual edge detection controller optimized for Raspberry Pi Zero W

import threading

# OpenCV + numpy are the biggest RAM consumers in this app (~80-150 MB on a
# Pi Zero W) and were historically imported at startup, starving the 512 MB
# board until the Wi-Fi firmware failed to allocate memory.  They are now
# imported ONLY when a frame is actually analysed (i.e. autonomous mode is
# running); manual driving never loads them.  The import runs in a background
# thread, and the first analysis waits (bounded) for it instead of silently
# reporting CLEAR while cv2 is still importing.
OPENCV_AVAILABLE = False
cv2 = None
np = None

_cv_load_done = threading.Event()
_cv_load_lock = threading.Lock()
_cv_loader_started = False


def _load_opencv():
    global cv2, np, OPENCV_AVAILABLE
    try:
        import cv2 as _cv2
        import numpy as _np
        # Publish both modules before flipping the flag so any thread that
        # sees OPENCV_AVAILABLE=True can use cv2 and np immediately.
        with _cv_load_lock:
            cv2 = _cv2
            np = _np
            OPENCV_AVAILABLE = True
        print("📷 OpenCV loaded for Pi Zero W visual edge detection.")
    except Exception as exc:  # ImportError, or broken native libs
        OPENCV_AVAILABLE = False
        print(f"⚠️ OpenCV unavailable ({exc}). Vision analysis disabled.")
    finally:
        _cv_load_done.set()


def ensure_opencv_loaded(timeout=15.0):
    """Start the background OpenCV import (once) and wait for it (bounded).

    Called before the autonomous routine starts moving so obstacle vision is
    ready while the rover is still stationary.  Returns True when OpenCV is
    usable.  Safe to call repeatedly: after the loader finishes this is a
    no-op, whether the load succeeded or not.
    """
    global _cv_loader_started
    with _cv_load_lock:
        if not _cv_loader_started:
            _cv_loader_started = True
            threading.Thread(target=_load_opencv, daemon=True, name="opencv-loader").start()
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

def analyze_visual_edges(jpg_bytes=None):
    """
    Lightweight Canny Edge & Barrier Detection optimized for Raspberry Pi Zero W (ARMv6 512MB RAM).
    Downscales frame to 160x120 and evaluates lower ground ROI edge density.
    """
    global vision_status, last_analyzed_frame_id, last_analyzed_result

    frame_to_process = jpg_bytes if jpg_bytes is not None else latest_jpeg_frame

    # No camera frame -> nothing to analyse, so never pay for OpenCV here.
    # This is the common case while driving manually.
    if not frame_to_process:
        vision_status = "CLEAR"
        return "CLEAR", 0, 0

    # A real analysis is due: make sure OpenCV is loaded (bounded wait for the
    # background import instead of blind-reporting CLEAR while it runs).
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
        np_arr = np.frombuffer(frame_to_process, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            vision_status = "CLEAR"
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
        EDGE_BARRIER_THRESHOLD = 1400
        
        if total_density > EDGE_BARRIER_THRESHOLD:
            if left_density > right_density:
                status = "OBSTACLE_LEFT"
            else:
                status = "OBSTACLE_RIGHT"
        else:
            status = "CLEAR"
            
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
