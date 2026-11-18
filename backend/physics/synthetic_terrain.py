"""
Realistic 3D Synthetic Terrain & Airborne LiDAR Simulator
Phase 04: Generates realistic multi-scale topography with Perlin-like noise, mountain peaks, valleys, and canopy clutter.
Author: Principal Software Architect
"""

import math
import numpy as np
from typing import Tuple, Dict, Any

class SyntheticTerrainGenerator:
    """
    Generates realistic elevation maps (DEM) and airborne LiDAR point clouds.
    Combines low-frequency tectonic ridges, rolling hills, and high-frequency vegetation canopy.
    """
    def __init__(self, seed: int = 42):
        self.rng = np.random.RandomState(seed)

    def generate_dem(
        self,
        rows: int,
        cols: int,
        base_elevation_m: float = 120.0,
        max_mountain_height_m: float = 450.0,
        roughness: float = 0.5
    ) -> np.ndarray:
        """
        Synthesizes a realistic Digital Elevation Model (DEM) using harmonic Fourier surfaces.
        """
        # Coordinate grids in normalized [-1, 1]
        y, x = np.mgrid[-1:1:complex(0, rows), -1:1:complex(0, cols)]
        
        # Primary Mountain Ridge
        ridge = max_mountain_height_m * np.exp(-((x - 0.2)**2 + (y + 0.3)**2) / 0.35)
        
        # Secondary Hill Range
        hills = (max_mountain_height_m * 0.4) * np.sin(3.0 * x) * np.cos(3.0 * y)
        
        # Valley Depression
        valley = -80.0 * np.exp(-((x + 0.4)**2 + (y - 0.2)**2) / 0.15)
        
        # Micro-topography / Perlin harmonic approximation
        micro = 15.0 * np.sin(10.0 * x + 5.0 * y) * np.cos(8.0 * y)
        
        dem = base_elevation_m + ridge + hills + valley + micro
        # Ensure positive ground elevations
        dem = np.maximum(dem, base_elevation_m * 0.5)
        return dem

    def simulate_lidar_scan(
        self,
        dem: np.ndarray,
        drone_altitude_agl_m: float = 60.0,
        canopy_density: float = 0.35,
        canopy_height_m: float = 12.0,
        sensor_noise_sigma: float = 0.15
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates raw airborne LiDAR rangefinder returns.
        
        Returns:
            raw_ranges: Measured distances from UAV to first reflection (canopy or ground)
            uav_altitudes: Absolute UAV altitude above sea level (AMSL)
        """
        rows, cols = dem.shape
        uav_altitudes = dem + drone_altitude_agl_m
        
        # Generate canopy/vegetation clutter mask
        has_canopy = (self.rng.rand(rows, cols) < canopy_density).astype(float)
        canopy_offset = has_canopy * self.rng.uniform(2.0, canopy_height_m, size=(rows, cols))
        
        # Gaussian rangefinder measurement noise
        noise = self.rng.normal(0.0, sensor_noise_sigma, size=(rows, cols))
        
        # Target reflection surface: Ground + Canopy
        reflected_surface = dem + canopy_offset + noise
        
        # Measured range = UAV AMSL - Reflected Surface
        raw_ranges = uav_altitudes - reflected_surface
        return raw_ranges, uav_altitudes
