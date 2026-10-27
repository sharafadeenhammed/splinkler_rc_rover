"""Work-area & map-origin persistence (JSON file) + local pose tracking.

History/run/telemetry logging was removed on purpose: the rover no longer
writes navigation events anywhere.  Only what the dashboard needs to survive
a reboot is persisted — the saved work areas and the map origin — in a small
atomic JSON file.
"""

import json
import math
import os
import threading
import time

from .pose_fusion import (
    GPS_FUSION_ALPHA,
    GPS_HEADING_BETA,
    GPS_MAX_JUMP_M,
    angle_lerp,
    geo_to_local,
    gps_fix_usable,
    gps_heading_usable,
    normalize_angle_rad,
)

STORE_PATH = os.environ.get(
    "ROVER_AREAS_PATH",
    os.path.join(os.path.dirname(__file__), "..", "work_areas.json"),
)

_EMPTY_STORE = {"next_area_id": 1, "map_origin": None, "areas": []}


class NavigationDatabase:
    """Tiny atomic JSON store for work areas + map origin."""

    def __init__(self, path=STORE_PATH):
        self.path = os.path.abspath(path)
        self.lock = threading.RLock()
        self._data = self._load()

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            if not isinstance(data, dict) or not isinstance(data.get("areas"), list):
                raise ValueError("unexpected store shape")
            data.setdefault("next_area_id", 1)
            data.setdefault("map_origin", None)
            return data
        except FileNotFoundError:
            return json.loads(json.dumps(_EMPTY_STORE))
        except (ValueError, TypeError, OSError) as exc:
            print(f"⚠️ work area store unreadable ({exc}); starting empty")
            return json.loads(json.dumps(_EMPTY_STORE))

    def _flush_locked(self):
        """Atomic write so a power cut can never leave a half-written file."""
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(self._data, fh, indent=1)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.path)

    # ── work areas ─────────────────────────────────────────────────────

    @staticmethod
    def _new_area(area_id, name, points, map_points, origin):
        clean = [[float(p[0]), float(p[1])] for p in points]
        min_x = min(p[0] for p in clean)
        max_x = max(p[0] for p in clean)
        min_y = min(p[1] for p in clean)
        max_y = max(p[1] for p in clean)
        return {
            "id": area_id,
            "name": str(name or "Work area"),
            "min_x": min_x,
            "min_y": min_y,
            "max_x": max_x,
            "max_y": max_y,
            "polygon": clean,
            "map_polygon": [[float(p[0]), float(p[1])] for p in (map_points or [])],
            "origin_lat": float(origin[0]) if origin else None,
            "origin_lon": float(origin[1]) if origin else None,
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "active": 1,
        }

    def _insert_locked(self, area):
        for other in self._data["areas"]:
            other["active"] = 0
        area["id"] = self._data["next_area_id"]
        self._data["next_area_id"] += 1
        self._data["areas"].append(area)
        self._flush_locked()
        return area

    def save_area(self, name, min_x, min_y, max_x, max_y):
        min_x, max_x = sorted((float(min_x), float(max_x)))
        min_y, max_y = sorted((float(min_y), float(max_y)))
        if min_x == max_x or min_y == max_y:
            raise ValueError("A work area must have non-zero width and height")
        rectangle = [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]]
        with self.lock:
            return self._insert_locked(self._new_area(0, name, rectangle, None, None))

    def save_polygon_area(self, name, points, map_points=None, origin=None):
        """Save local metre points and optional geographic map coordinates."""
        clean = []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("Each polygon point must contain x and y")
            clean.append([float(point[0]), float(point[1])])
        if len(clean) < 3:
            raise ValueError("Draw at least three points for the work area")
        min_x = min(p[0] for p in clean)
        max_x = max(p[0] for p in clean)
        min_y = min(p[1] for p in clean)
        max_y = max(p[1] for p in clean)
        if min_x == max_x or min_y == max_y:
            raise ValueError("The work area must have width and height")
        if map_points is not None and len(map_points) != len(clean):
            raise ValueError("Map points must match boundary points")
        with self.lock:
            return self._insert_locked(self._new_area(0, name, clean, map_points, origin))

    def get_area(self, area_id):
        with self.lock:
            for area in self._data["areas"]:
                if area["id"] == int(area_id):
                    return area
        return None

    def list_areas(self):
        with self.lock:
            return list(reversed(self._data["areas"]))

    def active_area(self):
        with self.lock:
            for area in reversed(self._data["areas"]):
                if area.get("active"):
                    return area
        return None

    # ── map origin ─────────────────────────────────────────────────────

    def save_map_origin(self, latitude, longitude):
        origin = [float(latitude), float(longitude)]
        with self.lock:
            self._data["map_origin"] = origin
            self._flush_locked()
        return origin

    def map_origin(self):
        with self.lock:
            origin = self._data.get("map_origin")
            if isinstance(origin, (list, tuple)) and len(origin) == 2:
                return [float(origin[0]), float(origin[1])]
        return None


class PoseTracker:
    def __init__(self, wheel_diameter_m, wheel_track_m, ticks_per_rev):
        self.circumference = math.pi * wheel_diameter_m
        self.track = wheel_track_m
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.heading_rad = 0.0
        self._x_odom = self._y_odom = 0.0
        self._last_ticks = None
        self.lock = threading.RLock()
        self.origin_lat = None
        self.origin_lon = None
        self._gps_position_active = False
        self._gps_heading_active = False
        self._fusion_note = "odometry"

    def set_map_origin(self, latitude, longitude):
        with self.lock:
            self.origin_lat = float(latitude)
            self.origin_lon = float(longitude)

    def clear_map_origin(self):
        with self.lock:
            self.origin_lat = self.origin_lon = None
            self._gps_position_active = False
            self._gps_heading_active = False
            self._fusion_note = "odometry"

    def reset(self, x=0.0, y=0.0, heading_rad=0.0):
        with self.lock:
            self.x = self._x_odom = float(x)
            self.y = self._y_odom = float(y)
            self.heading_rad = normalize_angle_rad(float(heading_rad))
            self._last_ticks = None
            self._gps_position_active = False
            self._gps_heading_active = False
            self._fusion_note = "origin_reset"

    def _apply_gps_fusion(self, gps):
        if not gps_fix_usable(gps):
            self._gps_position_active = False
            return
        if self.origin_lat is None or self.origin_lon is None:
            self._gps_position_active = False
            return

        x_gps, y_gps = geo_to_local(
            float(gps["lat"]), float(gps["lon"]), self.origin_lat, self.origin_lon,
        )
        jump = math.hypot(x_gps - self.x, y_gps - self.y)
        if self._gps_position_active and jump > GPS_MAX_JUMP_M:
            self._fusion_note = "gps_spike_rejected"
            return

        alpha = GPS_FUSION_ALPHA
        self.x = (1.0 - alpha) * self.x + alpha * x_gps
        self.y = (1.0 - alpha) * self.y + alpha * y_gps
        self._gps_position_active = True
        self._fusion_note = "gps_encoders_imu"

        if gps_heading_usable(gps):
            course_rad = math.radians(float(gps["course_deg"]))
            self.heading_rad = angle_lerp(self.heading_rad, course_rad, GPS_HEADING_BETA)
            self._gps_heading_active = True

    def update(self, left_ticks, right_ticks, imu=None, gps=None, map_origin=None):
        if map_origin and len(map_origin) == 2:
            self.set_map_origin(map_origin[0], map_origin[1])

        with self.lock:
            if self._last_ticks is not None:
                dl = (left_ticks - self._last_ticks[0]) * self.circumference / self.ticks_per_rev
                dr = (right_ticks - self._last_ticks[1]) * self.circumference / self.ticks_per_rev
                distance = (dl + dr) / 2.0
                encoder_heading = self.heading_rad + (dr - dl) / self.track
                if imu and imu.get("available"):
                    # MPU6050 yaw (magnetometer can replace GPS course blend later).
                    self.heading_rad = normalize_angle_rad(math.radians(imu["heading_deg"]))
                else:
                    self.heading_rad = normalize_angle_rad(encoder_heading)
                mid_heading = normalize_angle_rad((self.heading_rad + encoder_heading) / 2.0)
                self._x_odom += distance * math.cos(mid_heading)
                self._y_odom += distance * math.sin(mid_heading)
                self.x = self._x_odom
                self.y = self._y_odom
                self._fusion_note = "odometry"

            self._last_ticks = (left_ticks, right_ticks)
            if gps:
                self._apply_gps_fusion(gps)
            return self._snapshot_locked()

    def _snapshot_locked(self):
        return {
            "x_m": self.x,
            "y_m": self.y,
            "heading_deg": math.degrees(self.heading_rad),
            "fusion": {
                "mode": self._fusion_note,
                "gps_position": self._gps_position_active,
                "gps_heading": self._gps_heading_active,
                "x_odom_m": self._x_odom,
                "y_odom_m": self._y_odom,
            },
        }

    def snapshot(self):
        with self.lock:
            return self._snapshot_locked()


DB = NavigationDatabase()

__all__ = ["DB", "PoseTracker", "NavigationDatabase"]
