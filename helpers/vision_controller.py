# Camera visual edge detection controller optimized for Raspberry Pi Zero W

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
    print("📷 OpenCV loaded for Pi Zero W visual edge detection.")
except ImportError:
    OPENCV_AVAILABLE = False
    print("⚠️ OpenCV not installed. Vision analysis disabled.")

vision_status = "CLEAR"
latest_jpeg_frame = None

def set_latest_frame(jpeg_bytes):
    global latest_jpeg_frame
    latest_jpeg_frame = jpeg_bytes

def get_latest_frame():
    return latest_jpeg_frame

def analyze_visual_edges(jpg_bytes=None):
    """
    Lightweight Canny Edge & Barrier Detection optimized for Raspberry Pi Zero W (ARMv6 512MB RAM).
    Downscales frame to 160x120 and evaluates lower ground ROI edge density.
    """
    global vision_status
    frame_to_process = jpg_bytes if jpg_bytes is not None else latest_jpeg_frame

    if not OPENCV_AVAILABLE or not frame_to_process:
        vision_status = "CLEAR"
        return "CLEAR", 0, 0
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
                vision_status = "OBSTACLE_LEFT"
                return "OBSTACLE_LEFT", left_density, right_density
            else:
                vision_status = "OBSTACLE_RIGHT"
                return "OBSTACLE_RIGHT", left_density, right_density
                
        vision_status = "CLEAR"
        return "CLEAR", left_density, right_density
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
    "get_vision_status"
]
