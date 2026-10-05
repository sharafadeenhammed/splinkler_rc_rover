"""SQLite persistence and local pose tracking for the rover."""

import math
import json
import os
import sqlite3
import threading
import time
from contextlib import contextmanager


DB_PATH = os.environ.get("ROVER_DB_PATH", os.path.join(os.path.dirname(__file__), "..", "rover.db"))
print(" ---- ---- ---- db path -- ", DB_PATH)


class NavigationDatabase:
    def __init__(self, path=DB_PATH):
        self.path = os.path.abspath(path)
        self.lock = threading.RLock()
        self._init_schema()

    def _connect(self):
        conn = sqlite3.connect(self.path, timeout=10)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        return conn

    @contextmanager
    def _connection(self):
        conn = self._connect()
        try:
            yield conn
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    def _init_schema(self):
        with self.lock, self._connection() as conn:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS work_areas (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    min_x REAL NOT NULL, min_y REAL NOT NULL,
                    max_x REAL NOT NULL, max_y REAL NOT NULL,
                    polygon_json TEXT,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    active INTEGER NOT NULL DEFAULT 0
                );
                CREATE TABLE IF NOT EXISTS rover_runs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    area_id INTEGER,
                    started_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    ended_at TEXT,
                    status TEXT NOT NULL DEFAULT 'running',
                    FOREIGN KEY(area_id) REFERENCES work_areas(id)
                );
                CREATE TABLE IF NOT EXISTS telemetry (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    run_id INTEGER,
                    recorded_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    x REAL NOT NULL, y REAL NOT NULL, heading_deg REAL NOT NULL,
                    left_ticks INTEGER NOT NULL, right_ticks INTEGER NOT NULL,
                    pitch_deg REAL, roll_deg REAL, gyro_z_dps REAL,
                    FOREIGN KEY(run_id) REFERENCES rover_runs(id)
                );
            """)
            columns = {row[1] for row in conn.execute("PRAGMA table_info(work_areas)")}
            if "polygon_json" not in columns:
                conn.execute("ALTER TABLE work_areas ADD COLUMN polygon_json TEXT")

    @staticmethod
    def _area_dict(row):
        if not row:
            return None
        area = dict(row)
        try:
            area["polygon"] = json.loads(area.get("polygon_json") or "[]")
        except (TypeError, ValueError):
            area["polygon"] = []
        area.pop("polygon_json", None)
        return area

    def save_area(self, name, min_x, min_y, max_x, max_y):
        min_x, max_x = sorted((float(min_x), float(max_x)))
        min_y, max_y = sorted((float(min_y), float(max_y)))
        if min_x == max_x or min_y == max_y:
            raise ValueError("A work area must have non-zero width and height")
        with self.lock, self._connection() as conn:
            conn.execute("UPDATE work_areas SET active = 0")
            cur = conn.execute(
                "INSERT INTO work_areas(name,min_x,min_y,max_x,max_y,polygon_json,active) VALUES(?,?,?,?,?,?,1)",
                (str(name or "Work area"), min_x, min_y, max_x, max_y,
                 json.dumps([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]])),
            )
            area_id = cur.lastrowid
        return self.get_area(area_id)

    def save_polygon_area(self, name, points):
        """Save a user-drawn polygon and make it the active work area."""
        clean = []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("Each polygon point must contain x and y")
            clean.append([float(point[0]), float(point[1])])
        if len(clean) < 3:
            raise ValueError("Draw at least three points for the work area")
        min_x = min(point[0] for point in clean)
        max_x = max(point[0] for point in clean)
        min_y = min(point[1] for point in clean)
        max_y = max(point[1] for point in clean)
        if min_x == max_x or min_y == max_y:
            raise ValueError("The work area must have width and height")
        with self.lock, self._connection() as conn:
            conn.execute("UPDATE work_areas SET active = 0")
            cur = conn.execute(
                "INSERT INTO work_areas(name,min_x,min_y,max_x,max_y,polygon_json,active) VALUES(?,?,?,?,?,?,1)",
                (str(name or "Work area"), min_x, min_y, max_x, max_y, json.dumps(clean)),
            )
            area_id = cur.lastrowid
        return self.get_area(area_id)

    def get_area(self, area_id):
        with self.lock, self._connection() as conn:
            row = conn.execute("SELECT * FROM work_areas WHERE id = ?", (area_id,)).fetchone()
            return self._area_dict(row)

    def list_areas(self):
        with self.lock, self._connection() as conn:
            return [self._area_dict(row) for row in conn.execute("SELECT * FROM work_areas ORDER BY id DESC")]

    def active_area(self):
        with self.lock, self._connection() as conn:
            row = conn.execute("SELECT * FROM work_areas WHERE active = 1 ORDER BY id DESC LIMIT 1").fetchone()
            return self._area_dict(row)

    def start_run(self, area_id=None):
        with self.lock, self._connection() as conn:
            cur = conn.execute("INSERT INTO rover_runs(area_id) VALUES(?)", (area_id,))
            return cur.lastrowid

    def finish_run(self, run_id, status="complete"):
        with self.lock, self._connection() as conn:
            conn.execute("UPDATE rover_runs SET ended_at = CURRENT_TIMESTAMP, status = ? WHERE id = ?", (status, run_id))

    def record_telemetry(self, run_id, pose, imu, left_ticks, right_ticks):
        with self.lock, self._connection() as conn:
            conn.execute(
                "INSERT INTO telemetry(run_id,x,y,heading_deg,left_ticks,right_ticks,pitch_deg,roll_deg,gyro_z_dps) VALUES(?,?,?,?,?,?,?,?,?)",
                (run_id, pose["x_m"], pose["y_m"], pose["heading_deg"], left_ticks, right_ticks,
                 imu.get("pitch_deg"), imu.get("roll_deg"), imu.get("gyro_z_dps")),
            )

    def recent_telemetry(self, run_id=None, limit=500):
        limit = max(1, min(5000, int(limit)))
        with self.lock, self._connection() as conn:
            if run_id is None:
                rows = conn.execute(
                    "SELECT * FROM telemetry ORDER BY id DESC LIMIT ?", (limit,)
                ).fetchall()
            else:
                rows = conn.execute(
                    "SELECT * FROM telemetry WHERE run_id = ? ORDER BY id DESC LIMIT ?",
                    (run_id, limit),
                ).fetchall()
            return [dict(row) for row in reversed(rows)]

    def recent_runs(self, limit=50):
        limit = max(1, min(500, int(limit)))
        with self.lock, self._connection() as conn:
            return [dict(row) for row in conn.execute(
                "SELECT * FROM rover_runs ORDER BY id DESC LIMIT ?", (limit,)
            )]


class PoseTracker:
    def __init__(self, database, wheel_diameter_m, wheel_track_m, ticks_per_rev):
        self.db = database
        self.circumference = math.pi * wheel_diameter_m
        self.track = wheel_track_m
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.heading_rad = 0.0
        self._last_ticks = None
        self._last_save = 0.0
        self.run_id = None
        self.lock = threading.RLock()

    def reset(self, x=0.0, y=0.0, heading_rad=0.0):
        with self.lock:
            self.x, self.y, self.heading_rad = float(x), float(y), float(heading_rad)
            self._last_ticks = None

    def update(self, left_ticks, right_ticks, imu=None):
        with self.lock:
            if self._last_ticks is not None:
                dl = (left_ticks - self._last_ticks[0]) * self.circumference / self.ticks_per_rev
                dr = (right_ticks - self._last_ticks[1]) * self.circumference / self.ticks_per_rev
                distance = (dl + dr) / 2.0
                encoder_heading = self.heading_rad + (dr - dl) / self.track
                if imu and imu.get("available"):
                    # IMU heading is relative to its reset; use it for yaw while
                    # encoders provide distance and position integration.
                    self.heading_rad = math.radians(imu["heading_deg"])
                else:
                    self.heading_rad = encoder_heading
                mid_heading = (self.heading_rad + encoder_heading) / 2.0
                self.x += distance * math.cos(mid_heading)
                self.y += distance * math.sin(mid_heading)
            self._last_ticks = (left_ticks, right_ticks)
            pose = self.snapshot()
        now = time.monotonic()
        # Position is updated at 20 Hz, but one durable sample per second is
        # enough for route history and keeps SQLite I/O small on a 512 MB Pi.
        if self.run_id and now - self._last_save >= 1.0:
            self.db.record_telemetry(self.run_id, pose, imu or {}, left_ticks, right_ticks)
            self._last_save = now
        return pose

    def snapshot(self):
        return {"x_m": self.x, "y_m": self.y, "heading_deg": math.degrees(self.heading_rad)}


DB = NavigationDatabase()

__all__ = ["DB", "PoseTracker", "NavigationDatabase"]
