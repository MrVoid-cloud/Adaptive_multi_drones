"""
Computational UAV Flight Dynamics, LiDAR Terrain Engine & RTL Battery Model
Phase 04: Geodesic Haversine Navigation, LiDAR Canopy Ground Filtering, 
Terrain Roughness Factor (tau_j in [1.0, 3.0]), and 3D Aerodynamic Energy Consumption.
Author: Principal Software Architect
"""

import math
from typing import Tuple, Dict, Any, List, Optional
import numpy as np

EARTH_RADIUS_KM = 6371.0
EARTH_RADIUS_M = 6371000.0

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Computes exact geodesic surface distance (in meters) between two points on the WGS84 sphere.
    Avoids Euclidean planar distortion over large GIS search spaces.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    sin_dphi = math.sin(delta_phi / 2.0)
    sin_dlam = math.sin(delta_lambda / 2.0)

    a = sin_dphi * sin_dphi + math.cos(phi1) * math.cos(phi2) * sin_dlam * sin_dlam
    # Clamping for floating-point safety
    a = min(1.0, max(0.0, a))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    return EARTH_RADIUS_M * c


class LiDARTerrainProcessor:
    """
    Airborne LiDAR signal processing unit.
    Converts laser time-of-flight pulses to ground elevation DEM,
    removes vegetation canopy clutter, and derives the terrain roughness factor tau_j.
    """
    TERRAIN_CLASSES = {
        "OPEN_GROUND": (1.0, 1.30, "Flat plains, paved zones, and open grass."),
        "ROLLING_DESERT": (1.30, 1.80, "Undulating sand dunes, mild agricultural knolls."),
        "DENSE_FOREST": (1.80, 2.40, "Dense vegetation canopy, irregular surface contours."),
        "RUGGED_MOUNTAINOUS": (2.40, 3.00, "Steep cliffs, rocky ridges, high vertical elevation gradients.")
    }

    @staticmethod
    def extract_elevation(uav_altitude_amsl_m: Any, laser_range_m: Any) -> Any:
        """
        Z_terrain = H_drone - h_laser
        Converts sensor slant/downward distance to absolute terrain elevation.
        Works for both scalar float and NumPy arrays.
        """
        if isinstance(uav_altitude_amsl_m, np.ndarray) or isinstance(laser_range_m, np.ndarray):
            return np.maximum(0.0, np.array(uav_altitude_amsl_m) - np.array(laser_range_m))
        return max(0.0, float(uav_altitude_amsl_m) - float(laser_range_m))

    @classmethod
    def filter_canopy_noise(
        cls,
        raw_dem: np.ndarray,
        window_size: int = 3,
        canopy_height_threshold_m: float = 2.5
    ) -> np.ndarray:
        """
        Applies a Morphological Outlier & Local Ground Filter to strip canopy reflections
        (vegetation noise) while preserving underlying topographic slope.
        If a return exceeds the local median by more than the canopy threshold, it is treated
        as vegetation clutter and clamped to the ground envelope.
        """
        rows, cols = raw_dem.shape
        filtered_dem = np.copy(raw_dem)
        pad = window_size // 2

        # Pad matrix with edge reflections
        padded = np.pad(raw_dem, pad, mode='edge')

        for r in range(rows):
            for c in range(cols):
                window = padded[r:r + window_size, c:c + window_size]
                local_med = float(np.median(window))
                val = raw_dem[r, c]
                # If reading is significantly higher than local surface, it's canopy vegetation
                if val - local_med > canopy_height_threshold_m:
                    filtered_dem[r, c] = local_med
                else:
                    filtered_dem[r, c] = val

        return filtered_dem

    @classmethod
    def compute_terrain_factor(
        cls,
        elevations: np.ndarray,
        cell_size_m: float
    ) -> Tuple[float, str]:
        """
        Computes the terrain difficulty multiplier tau_j in [1.0, 3.0] and classifies terrain.
        
        Formula:
            Slope: S_j = (Z_max - Z_min) / L_cell
            Roughness: sigma_Z = std_dev(Z)
            tau_j = 1.0 + min(2.0, 1.2 * S_j + 0.8 * (sigma_Z / 15.0))
        """
        if elevations.size == 0 or np.isnan(elevations).all():
            return 1.0, "OPEN_GROUND"

        z_min = float(np.min(elevations))
        z_max = float(np.max(elevations))
        delta_z = z_max - z_min
        slope = delta_z / max(1.0, cell_size_m)
        roughness_sigma = float(np.std(elevations))

        # Scaling formula mapped strictly to [1.0, 3.0]
        factor_increment = (1.2 * min(1.0, slope)) + (0.8 * min(1.0, roughness_sigma / 15.0))
        tau_j = round(float(1.0 + min(2.0, factor_increment)), 2)

        # Classification
        if tau_j < 1.30:
            classification = "OPEN_GROUND"
        elif tau_j < 1.80:
            classification = "ROLLING_DESERT"
        elif tau_j < 2.40:
            classification = "DENSE_FOREST"
        else:
            classification = "RUGGED_MOUNTAINOUS"

        return tau_j, classification


class UAVPhysicsEngine:
    """
    Aerodynamic Flight Dynamics, Energy Expenditure, and Dynamic RTL Safety Reserve.
    Accurately accounts for 3D trajectory profiles, vertical climb penalties, and ambient wind.
    """
    def __init__(
        self,
        nominal_mass_kg: float = 2.4,           # Quadrotor AUW (All-Up Weight)
        battery_capacity_wh: float = 120.0,     # 6S LiPo ~5400mAh = 120 Wh
        cruise_speed_mps: float = 14.0,         # Optimal cruise speed (m/s)
        climb_speed_mps: float = 4.0,           # Vertical climb rate (m/s)
        descent_speed_mps: float = 3.0,         # Vertical descent rate (m/s)
        p_hover_watts: float = 210.0,           # Baseline hovering power
        p_cruise_watts: float = 175.0,          # Aerodynamic forward cruise power
        climb_power_factor: float = 1.65,       # Multiplier for vertical ascent motor thrust
        critical_landing_reserve_pct: float = 15.0, # Mandatory deadman landing reserve (%)
        wind_safety_factor: float = 1.25        # 25% safety overhead for headwinds
    ):
        self.mass_kg = nominal_mass_kg
        self.battery_capacity_wh = battery_capacity_wh
        self.v_cruise = cruise_speed_mps
        self.v_climb = climb_speed_mps
        self.v_descent = descent_speed_mps
        self.p_hover = p_hover_watts
        self.p_cruise = p_cruise_watts
        self.p_climb = self.p_hover * climb_power_factor
        self.p_descent = self.p_hover * 0.75
        self.critical_reserve_pct = critical_landing_reserve_pct
        self.wind_safety_factor = wind_safety_factor

    def calculate_transition_cost(
        self,
        start_lat: float,
        start_lon: float,
        start_alt_m: float,
        target_lat: float,
        target_lon: float,
        target_alt_m: float,
        terrain_factor_tau: float = 1.0,
        wind_vector_mps: Tuple[float, float] = (0.0, 0.0) # (wind_speed, wind_bearing_deg)
    ) -> Dict[str, Any]:
        """
        Calculates realistic 3D flight metrics between two coordinates.
        Takes into account geodesic distance, elevation delta (climb energy penalty), 
        terrain friction multiplier tau_j, and wind vector.
        """
        d_horiz_m = haversine_distance(start_lat, start_lon, target_lat, target_lon)
        delta_z = target_alt_m - start_alt_m
        d_3d_m = math.sqrt(d_horiz_m * d_horiz_m + delta_z * delta_z)

        # 1. Effective forward groundspeed considering wind
        wind_speed, wind_dir = wind_vector_mps
        # Calculate flight azimuth (bearing)
        lat1_rad = math.radians(start_lat)
        lat2_rad = math.radians(target_lat)
        dlon_rad = math.radians(target_lon - start_lon)
        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
        flight_bearing_deg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

        # Wind component along flight direction: cos(wind_bearing - flight_bearing)
        angle_diff = math.radians(wind_dir - flight_bearing_deg)
        headwind_mps = wind_speed * math.cos(angle_diff)
        ground_speed_mps = max(4.0, self.v_cruise - headwind_mps)

        # 2. Time decomposition
        t_horiz_sec = d_horiz_m / ground_speed_mps
        
        # Vertical climb / descent times
        if delta_z > 0:
            t_vert_sec = delta_z / self.v_climb
            p_vert_active = self.p_climb
        else:
            t_vert_sec = abs(delta_z) / self.v_descent
            p_vert_active = self.p_descent

        # Synchronous flight motion time
        total_time_sec = max(t_horiz_sec, t_vert_sec)

        # 3. Energy Consumption Model (Joules & Watt-hours)
        # Horizontal energy modulated by terrain difficulty factor tau_j
        e_horiz_joules = (self.p_cruise * terrain_factor_tau) * t_horiz_sec
        e_vert_joules = p_vert_active * t_vert_sec
        total_energy_joules = e_horiz_joules + e_vert_joules
        total_energy_wh = total_energy_joules / 3600.0

        # Percentage of battery pack consumed
        battery_pct_cost = (total_energy_wh / self.battery_capacity_wh) * 100.0

        return {
            "distance_horizontal_m": round(d_horiz_m, 2),
            "distance_3d_m": round(d_3d_m, 2),
            "delta_z_m": round(delta_z, 2),
            "flight_time_sec": round(total_time_sec, 2),
            "energy_consumed_wh": round(total_energy_wh, 3),
            "battery_pct_cost": round(battery_pct_cost, 2),
            "effective_ground_speed_mps": round(ground_speed_mps, 2),
            "terrain_factor_tau": terrain_factor_tau
        }

    def compute_dynamic_rtl_reserve(
        self,
        current_lat: float,
        current_lon: float,
        base_lat: float,
        base_lon: float,
        current_battery_pct: float,
        headwind_estimate_mps: float = 4.0
    ) -> Dict[str, Any]:
        """
        Dynamically calculates the Return-to-Launch (RTL) safety battery threshold.
        If current_battery_pct <= rtl_threshold_pct, the UAV must immediately abort and RTL.
        
        Formula:
            B_RTL = min(100.0, 15.0% + (d_base / V_cruise) * (P_cruise / E_batt) * k_wind * 100%)
        """
        d_base_m = haversine_distance(current_lat, current_lon, base_lat, base_lon)
        
        # Flight time to base against headwind safety margin
        eff_rtl_speed = max(4.0, self.v_cruise - headwind_estimate_mps)
        t_return_sec = d_base_m / eff_rtl_speed

        # Return flight energy in Wh
        e_return_wh = (self.p_cruise * t_return_sec * self.wind_safety_factor) / 3600.0
        flight_battery_pct = (e_return_wh / self.battery_capacity_wh) * 100.0

        # Dynamic RTL Threshold = Return transit + Critical descent reserve (15%)
        rtl_threshold_pct = round(min(100.0, self.critical_reserve_pct + flight_battery_pct), 2)
        safe_margin_pct = round(current_battery_pct - rtl_threshold_pct, 2)
        must_trigger_rtl = current_battery_pct <= rtl_threshold_pct

        return {
            "distance_to_base_m": round(d_base_m, 2),
            "return_flight_time_sec": round(t_return_sec, 1),
            "current_battery_pct": round(current_battery_pct, 2),
            "rtl_threshold_pct": rtl_threshold_pct,
            "safety_margin_pct": safe_margin_pct,
            "must_trigger_rtl": must_trigger_rtl,
            "status": "CRITICAL_RTL_REQUIRED" if must_trigger_rtl else "SAFE_OPERATION"
        }
