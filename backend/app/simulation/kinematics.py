"""
Geodesic Kinematics & Forward Waypoint Propagation Engine
Phase 07: Real-Time Physics Simulator & UAV Kinematics
Author: Principal Software Architect
"""

import math
from typing import Tuple, Dict, Any
from app.physics.flight_physics import haversine_distance, EARTH_RADIUS_M

class GeodesicKinematics:
    """
    Computes great-circle forward motion, bearing, and step propagation
    to avoid waypoint overshooting during time-stepped simulation.
    """
    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculates initial forward bearing (azimuth in radians) from point 1 to point 2.
        """
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        bearing_rad = math.atan2(y, x)
        return (bearing_rad + 2.0 * math.pi) % (2.0 * math.pi)

    @classmethod
    def step_towards(
        cls,
        current_lat: float,
        current_lon: float,
        target_lat: float,
        target_lon: float,
        speed_mps: float,
        dt_seconds: float,
        arrival_threshold_m: float = 2.0
    ) -> Tuple[float, float, float, bool]:
        """
        Propagates UAV position towards target coordinate along the geodesic great circle.
        Guarantees ZERO overshooting: Clamps step distance to remaining distance.

        Returns:
            new_lat, new_lon, distance_covered_m, has_arrived
        """
        dist_rem = haversine_distance(current_lat, current_lon, target_lat, target_lon)

        # Arrival check within tolerance radius
        if dist_rem <= arrival_threshold_m:
            return target_lat, target_lon, dist_rem, True

        # Maximum step achievable in this delta time
        step_dist = speed_mps * dt_seconds

        # Anti-overshooting clamp
        if step_dist >= dist_rem:
            return target_lat, target_lon, dist_rem, True

        # Forward geodesic position calculation
        bearing = cls.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        phi1 = math.radians(current_lat)
        lam1 = math.radians(current_lon)
        delta = step_dist / EARTH_RADIUS_M

        sin_phi2 = math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(bearing)
        # Clamping
        sin_phi2 = min(1.0, max(-1.0, sin_phi2))
        phi2 = math.asin(sin_phi2)

        y = math.sin(bearing) * math.sin(delta) * math.cos(phi1)
        x = math.cos(delta) - math.sin(phi1) * sin_phi2
        delta_lambda = math.atan2(y, x)
        lam2 = lam1 + delta_lambda

        new_lat = math.degrees(phi2)
        new_lon = (math.degrees(lam2) + 540.0) % 360.0 - 180.0

        return new_lat, new_lon, step_dist, False
