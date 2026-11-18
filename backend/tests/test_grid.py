import sys
import math
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from app.gis.grid import GeoCell, GeodesicGridDiscretizer
from app.api.schemas.geo_schemas import BoundingBox, GeoPoint
from app.api.schemas.grid_schemas import GridGenerationRequest
from app.api.v1.grid_routes import (
    generate_geodesic_grid,
    get_active_grid_metadata,
    lookup_cell_by_coordinate
)

def test_sensor_fov_metric_discretization():
    print("[*] Testing Sensor FOV & Geodesic Calculation...")
    h = 60.0
    fov = 70.0
    overlap = 0.10
    
    # Expected footprint: 2 * 60 * tan(35 deg) * 0.90
    expected_l = 2.0 * h * math.tan(math.radians(35.0)) * 0.90
    
    grid = GeodesicGridDiscretizer(
        lat_min=33.30,
        lat_max=33.32,
        lon_min=44.35,
        lon_max=44.38,
        altitude_m=h,
        sensor_fov_deg=fov,
        overlap_ratio=overlap
    )
    
    assert abs(grid.cell_size_m - expected_l) < 0.01, f"Expected {expected_l}, got {grid.cell_size_m}"
    assert grid.delta_lat > 0
    assert grid.delta_lon > 0
    # At latitude ~33.3 degrees, 1 deg lon is smaller than 1 deg lat, so delta_lon > delta_lat
    assert grid.delta_lon > grid.delta_lat, "Delta Lon degrees should be larger than Delta Lat degrees due to cos(lat) factor"
    print(f"  [✓] Sensor footprint verified: L_cell = {grid.cell_size_m:.2f} m, delta_lat = {grid.delta_lat:.6f}°, delta_lon = {grid.delta_lon:.6f}°")

def test_polygon_ray_casting_mask():
    print("[*] Testing Polygon Ray Casting (PIP) Masking...")
    # Triangular ROI inside bounding box [33.30, 33.34] x [44.35, 44.40]
    poly_vertices = [
        (33.30, 44.35),
        (33.34, 44.35),
        (33.30, 44.40)
    ]
    
    grid = GeodesicGridDiscretizer(
        lat_min=33.30,
        lat_max=33.34,
        lon_min=44.35,
        lon_max=44.40,
        polygon_vertices=poly_vertices,
        manual_cell_size_m=100.0
    )
    
    assert grid.total_cells > 0
    assert grid.active_cell_count > 0
    assert grid.masked_cell_count > 0
    assert grid.active_cell_count + grid.masked_cell_count == grid.total_cells
    
    # Check top-right corner of bounding box (33.34, 44.40): should be outside triangular ROI!
    cell_top_right = grid.coord_to_cell(33.339, 44.399)
    assert cell_top_right is not None
    assert cell_top_right.is_masked_out is True, "Top-right corner must be masked out"
    
    # Check near bottom-left (33.305, 44.355): should be inside triangular ROI!
    cell_bottom_left = grid.coord_to_cell(33.305, 44.355)
    assert cell_bottom_left is not None
    assert cell_bottom_left.is_masked_out is False, "Bottom-left cell must be active"
    print(f"  [✓] Ray casting verified: {grid.active_cell_count} active cells, {grid.masked_cell_count} masked cells.")

def test_o1_spatial_indexing_and_neighbors():
    print("[*] Testing O(1) Spatial Indexing and Neighbor Graph...")
    grid = GeodesicGridDiscretizer(
        lat_min=33.30,
        lat_max=33.32,
        lon_min=44.35,
        lon_max=44.38,
        manual_cell_size_m=80.0
    )
    
    # Test arbitrary cell
    target_cell: GeoCell = grid.cells[len(grid.cells) // 2]
    lookup_cell = grid.coord_to_cell(target_cell.center_lat, target_cell.center_lon)
    
    assert lookup_cell is not None
    assert lookup_cell.cell_id == target_cell.cell_id
    assert lookup_cell.row == target_cell.row
    assert lookup_cell.col == target_cell.col
    print(f"  [✓] O(1) direct coordinate-to-cell mapped perfectly: Cell ID {lookup_cell.cell_id} at ({lookup_cell.row}, {lookup_cell.col}).")
    
    # Test neighbor connectivity
    neighbors_4 = grid.get_neighbors(lookup_cell.row, lookup_cell.col, diagonal=False)
    assert len(neighbors_4) in (2, 3, 4)
    neighbors_8 = grid.get_neighbors(lookup_cell.row, lookup_cell.col, diagonal=True)
    assert len(neighbors_8) in (3, 5, 8)
    print(f"  [✓] Spatial neighbor graph verified (4-way: {len(neighbors_4)}, 8-way: {len(neighbors_8)}).")

def test_geojson_export_and_api():
    print("[*] Testing GeoJSON Serialization & API Endpoint...")
    req = GridGenerationRequest(
        roi_type="RECTANGLE",
        bounds=BoundingBox(lat_min=33.30, lat_max=33.315, lon_min=44.35, lon_max=44.37),
        vertices=[],
        altitude_m=50.0,
        sensor_fov_deg=80.0,
        overlap_ratio=0.15
    )
    
    res = generate_geodesic_grid(req)
    assert res.status == "GRID_GENERATED_AND_INDEXED"
    assert res.metadata.total_cells > 0
    assert res.geojson["type"] == "FeatureCollection"
    assert len(res.geojson["features"]) == res.metadata.total_cells
    
    # Verify first feature format
    feat0 = res.geojson["features"][0]
    assert feat0["geometry"]["type"] == "Polygon"
    assert len(feat0["geometry"]["coordinates"][0]) == 5 # Closed 4-corner ring
    assert "cell_id" in feat0["properties"]
    
    # Verify cell lookup endpoint
    center_lat = (33.30 + 33.315) / 2.0
    center_lon = (44.35 + 44.37) / 2.0
    lookup_res = lookup_cell_by_coordinate(lat=center_lat, lon=center_lon)
    assert lookup_res["found"] is True
    assert "cell" in lookup_res
    print(f"  [✓] GeoJSON generated with {len(res.geojson['features'])} cells. API lookup endpoint verified.")

if __name__ == "__main__":
    test_sensor_fov_metric_discretization()
    test_polygon_ray_casting_mask()
    test_o1_spatial_indexing_and_neighbors()
    test_geojson_export_and_api()
    print("\n[SUCCESS] All Phase 03 Geodesic Grid & Spatial Indexing Tests Passed!")
