"""Geo ↔ local conversions and GPS quality checks for sensor fusion."""

import math
import os

EARTH_RADIUS_M = 6378137.0

GPS_FUSION_ALPHA = float(os.environ.get("GPS_FUSION_ALPHA", "0.08"))
GPS_MIN_SATS = int(os.environ.get("GPS_MIN_SATS", "6"))
GPS_HEADING_MIN_SPEED_KMH = float(os.environ.get("GPS_HEADING_MIN_SPEED_KMH", "1.5"))
GPS_HEADING_BETA = float(os.environ.get("GPS_HEADING_BETA", "0.04"))
GPS_MAX_JUMP_M = float(os.environ.get("GPS_MAX_JUMP_M", "12"))


def geo_to_local(lat, lon, origin_lat, origin_lon):
    """Local ENU metres: x east, y north."""
    origin_lat_r = math.radians(origin_lat)
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    origin_lon_r = math.radians(origin_lon)
    x = EARTH_RADIUS_M * math.cos(origin_lat_r) * (lon_r - origin_lon_r)
    y = EARTH_RADIUS_M * (lat_r - origin_lat_r)
    return x, y


def local_to_geo(x, y, origin_lat, origin_lon):
    origin_lat_r = math.radians(origin_lat)
    lat = origin_lat + (y / EARTH_RADIUS_M) * (180.0 / math.pi)
    lon = origin_lon + (x / (EARTH_RADIUS_M * math.cos(origin_lat_r))) * (180.0 / math.pi)
    return lat, lon


def normalize_angle_rad(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def course_to_heading_rad(course_deg):
    """NMEA true course (0° = north, clockwise) → world heading (0° = east, CCW).

    The pose and the dashboard's map work in the local ENU frame (x east,
    y north), where angles run counter-clockwise from +x.  A
    GPS course is a compass bearing, so it has to be turned a quarter turn the
    other way before it can be used as a heading.
    """
    return normalize_angle_rad(math.radians(90.0 - float(course_deg)))


def angle_lerp(from_rad, to_rad, weight):
    weight = max(0.0, min(1.0, float(weight)))
    delta = normalize_angle_rad(to_rad - from_rad)
    return normalize_angle_rad(from_rad + delta * weight)


def gps_fix_usable(gps, min_sats=None):
    if not gps or not gps.get("available") or not gps.get("fix"):
        return False
    if gps.get("lat") is None or gps.get("lon") is None:
        return False
    min_sats = GPS_MIN_SATS if min_sats is None else min_sats
    return int(gps.get("sats") or 0) >= min_sats


def gps_heading_usable(gps, min_speed_kmh=None):
    if not gps_fix_usable(gps):
        return False
    if gps.get("course_deg") is None:
        return False
    min_speed_kmh = GPS_HEADING_MIN_SPEED_KMH if min_speed_kmh is None else min_speed_kmh
    speed = float(gps.get("speed_kmh") or 0.0)
    return speed >= min_speed_kmh


__all__ = [
    "EARTH_RADIUS_M",
    "GPS_FUSION_ALPHA",
    "GPS_MIN_SATS",
    "GPS_HEADING_MIN_SPEED_KMH",
    "GPS_HEADING_BETA",
    "GPS_MAX_JUMP_M",
    "geo_to_local",
    "local_to_geo",
    "normalize_angle_rad",
    "angle_lerp",
    "course_to_heading_rad",
    "gps_fix_usable",
    "gps_heading_usable",
]
