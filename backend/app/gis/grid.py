"""
Computational GIS Engine: Geodesic Grid Discretizer & Spatial Indexer
Phase 03: Sensor-driven equal-area grid, Ray Casting (PIP), O(1) Spatial Lookups, and GeoJSON Export.
Author: Principal Software Architect
"""

import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
from pydantic import BaseModel, Field

# Constants for WGS84 Geoid
WGS84_A = 6378137.0          # Semi-major axis (meters)
WGS84_E2 = 0.00669437999014  # First eccentricity squared

class GeoCell:
    """
    Rich data-bearing spatial unit representing an atomic terrain search block.
    """
    __slots__ = (
        'cell_id', 'row', 'col', 'center_lat', 'center_lon', 
        'corners', 'area_m2', 'elevation_m', 'terrain_roughness', 
        'priority', 'is_masked_out', 'is_scanned', 'scanned_by_drone', 
        'scan_timestamp'
    )

    def __init__(
        self,
        cell_id: int,
        row: int,
        col: int,
        center_lat: float,
        center_lon: float,
        corners: List[Tuple[float, float]],
        area_m2: float,
        elevation_m: float = 0.0,
        terrain_roughness: float = 1.0,
        priority: float = 1.0,
        is_masked_out: bool = False
    ):
        self.cell_id = cell_id
        self.row = row
        self.col = col
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.corners = corners  # Closed polygon: [SW, NW, NE, SE, SW]
        self.area_m2 = area_m2
        self.elevation_m = elevation_m
        self.terrain_roughness = terrain_roughness  # Friction/energy cost multiplier
        self.priority = priority                    # 1.0 = normal, higher = high-priority search target
        self.is_masked_out = is_masked_out          # True if outside user-drawn polygon
        self.is_scanned = False
        self.scanned_by_drone: Optional[str] = None
        self.scan_timestamp: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "cell_id": self.cell_id,
            "row": self.row,
            "col": self.col,
            "center": {"lat": self.center_lat, "lon": self.center_lon},
            "area_m2": round(self.area_m2, 2),
            "elevation_m": round(self.elevation_m, 2),
            "terrain_roughness": round(self.terrain_roughness, 2),
            "priority": self.priority,
            "is_masked_out": self.is_masked_out,
            "is_scanned": self.is_scanned,
            "scanned_by_drone": self.scanned_by_drone,
        }

    def to_geojson_feature(self) -> Dict[str, Any]:
        """Exports cell polygon in standard RFC 7946 GeoJSON [lon, lat] format."""
        coordinates = [[round(lon, 7), round(lat, 7)] for lat, lon in self.corners]
        return {
            "type": "Feature",
            "id": self.cell_id,
            "geometry": {
                "type": "Polygon",
                "coordinates": [coordinates]
            },
            "properties": {
                "cell_id": self.cell_id,
                "row": self.row,
                "col": self.col,
                "center_lat": round(self.center_lat, 7),
                "center_lon": round(self.center_lon, 7),
                "elevation_m": round(self.elevation_m, 2),
                "terrain_roughness": round(self.terrain_roughness, 2),
                "priority": self.priority,
                "is_masked_out": self.is_masked_out,
                "is_scanned": self.is_scanned
            }
        }


class GeodesicGridDiscretizer:
    """
    High-performance Geodesic Discretizer & O(1) Spatial Indexer.
    Transforms arbitrary continuous GIS bounds into an aligned, sensor-calibrated grid.
    """
    def __init__(
        self,
        lat_min: float,
        lat_max: float,
        lon_min: float,
        lon_max: float,
        polygon_vertices: Optional[List[Tuple[float, float]]] = None,
        altitude_m: float = 60.0,
        sensor_fov_deg: float = 70.0,
        manual_cell_size_m: Optional[float] = None,
        overlap_ratio: float = 0.10
    ):
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.lon_min = lon_min
        self.lon_max = lon_max
        self.polygon_vertices = polygon_vertices or []
        self.altitude_m = altitude_m
        self.sensor_fov_deg = sensor_fov_deg
        self.overlap_ratio = max(0.0, min(0.8, overlap_ratio))

        # 1. Determine Metric Resolution (L_cell)
        if manual_cell_size_m and manual_cell_size_m > 0:
            self.cell_size_m = manual_cell_size_m
        else:
            # L_cell = 2 * h * tan(FOV / 2) * (1 - overlap)
            fov_rad = math.radians(sensor_fov_deg)
            raw_footprint = 2.0 * altitude_m * math.tan(fov_rad / 2.0)
            self.cell_size_m = raw_footprint * (1.0 - self.overlap_ratio)

        # 2. Compute Geodesic Steps (Delta Lat & Delta Lon with Cosine Correction)
        self.mean_lat = (lat_min + lat_max) / 2.0
        self.mean_lat_rad = math.radians(self.mean_lat)

        # Radius of curvature in prime vertical & meridian
        sin_lat = math.sin(self.mean_lat_rad)
        w = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        m_per_deg_lat = (math.pi / 180.0) * (WGS84_A * (1.0 - WGS84_E2) / (w * w * w))
        m_per_deg_lon = (math.pi / 180.0) * (WGS84_A / w) * math.cos(self.mean_lat_rad)

        self.delta_lat = self.cell_size_m / m_per_deg_lat
        self.delta_lon = self.cell_size_m / m_per_deg_lon

        # 3. Grid Dimensions (Rows and Columns)
        self.rows = max(1, int(math.ceil((lat_max - lat_min) / self.delta_lat)))
        self.cols = max(1, int(math.ceil((lon_max - lon_min) / self.delta_lon)))
        self.total_cells = self.rows * self.cols

        # 4. Dense Data Structures for Fast Lookup
        self.cells: List[GeoCell] = []
        self.grid_matrix: np.ndarray = np.empty((self.rows, self.cols), dtype=object)
        self.active_cell_count: int = 0
        self.masked_cell_count: int = 0

        self._build_grid()

    @staticmethod
    def point_in_polygon(point_lat: float, point_lon: float, polygon: List[Tuple[float, float]]) -> bool:
        """
        Ray Casting Algorithm (Jordan Curve Theorem).
        Evaluates whether (point_lat, point_lon) lies inside the polygon in O(V) time.
        """
        n = len(polygon)
        if n < 3:
            return True  # If no polygon or degenerate, accept all

        inside = False
        p1_lat, p1_lon = polygon[0]

        for i in range(1, n + 1):
            p2_lat, p2_lon = polygon[i % n]
            if point_lat > min(p1_lat, p2_lat):
                if point_lat <= max(p1_lat, p2_lat):
                    if point_lon <= max(p1_lon, p2_lon):
                        if p1_lat != p2_lat:
                            x_inters = (point_lat - p1_lat) * (p2_lon - p1_lon) / (p2_lat - p1_lat) + p1_lon
                            if p1_lon == p2_lon or point_lon <= x_inters:
                                inside = not inside
            p1_lat, p1_lon = p2_lat, p2_lon

        return inside

    def _build_grid(self):
        """Constructs cells, calculates corners, and applies the polygon mask."""
        has_polygon = len(self.polygon_vertices) >= 3
        cell_area = self.cell_size_m * self.cell_size_m

        cell_counter = 0
        # Build from North to South (Row 0 = Top/North) or South to North
        for r in range(self.rows):
            # lat starts from lat_max downwards (standard cartographic orientation)
            c_lat_top = self.lat_max - (r * self.delta_lat)
            c_lat_bot = c_lat_top - self.delta_lat
            center_lat = (c_lat_top + c_lat_bot) / 2.0

            for c in range(self.cols):
                c_lon_left = self.lon_min + (c * self.delta_lon)
                c_lon_right = c_lon_left + self.delta_lon
                center_lon = (c_lon_left + c_lon_right) / 2.0

                # 4-Corners: SW -> NW -> NE -> SE -> SW (closed ring for GIS)
                corners = [
                    (c_lat_bot, c_lon_left),
                    (c_lat_top, c_lon_left),
                    (c_lat_top, c_lon_right),
                    (c_lat_bot, c_lon_right),
                    (c_lat_bot, c_lon_left)
                ]

                # Check Polygon Mask
                is_masked_out = False
                if has_polygon:
                    inside = self.point_in_polygon(center_lat, center_lon, self.polygon_vertices)
                    is_masked_out = not inside

                if is_masked_out:
                    self.masked_cell_count += 1
                else:
                    self.active_cell_count += 1

                cell = GeoCell(
                    cell_id=cell_counter,
                    row=r,
                    col=c,
                    center_lat=center_lat,
                    center_lon=center_lon,
                    corners=corners,
                    area_m2=cell_area,
                    is_masked_out=is_masked_out
                )

                self.cells.append(cell)
                self.grid_matrix[r, c] = cell
                cell_counter += 1

    def coord_to_cell(self, lat: float, lon: float) -> Optional[GeoCell]:
        """
        O(1) Spatial Indexing: Direct continuous coordinate to discrete cell lookup.
        Instantaneous telemetry localization for live drones.
        """
        if lat < self.lat_min or lat > self.lat_max or lon < self.lon_min or lon > self.lon_max:
            return None

        # Row 0 is at lat_max
        r = int((self.lat_max - lat) / self.delta_lat)
        c = int((lon - self.lon_min) / self.delta_lon)

        # Edge clamp for boundaries
        r = min(max(0, r), self.rows - 1)
        c = min(max(0, c), self.cols - 1)

        return self.grid_matrix[r, c]

    def get_neighbors(self, row: int, col: int, diagonal: bool = False) -> List[GeoCell]:
        """Retrieves 4-connected or 8-connected valid neighbors for path planning."""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if diagonal:
            directions.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        neighbors = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                cell: GeoCell = self.grid_matrix[nr, nc]
                if not cell.is_masked_out:
                    neighbors.append(cell)
        return neighbors

    def to_geojson(self, include_masked: bool = True) -> Dict[str, Any]:
        """Exports the complete search grid as a standard GeoJSON FeatureCollection."""
        features = []
        for cell in self.cells:
            if not include_masked and cell.is_masked_out:
                continue
            features.append(cell.to_geojson_feature())

        return {
            "type": "FeatureCollection",
            "properties": {
                "rows": self.rows,
                "cols": self.cols,
                "total_cells": self.total_cells,
                "active_cells": self.active_cell_count,
                "masked_cells": self.masked_cell_count,
                "cell_size_m": round(self.cell_size_m, 2),
                "delta_lat_deg": self.delta_lat,
                "delta_lon_deg": self.delta_lon,
                "mean_latitude": self.mean_lat
            },
            "features": features
        }
