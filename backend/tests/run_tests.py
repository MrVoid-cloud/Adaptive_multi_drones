import sys
from pathlib import Path

backend_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_path))

from pydantic import ValidationError
from app.api.schemas.geo_schemas import (
    GeoPoint,
    BoundingBox,
    SearchSpaceROIRequest,
    SearchSpaceROIResponse
)
from app.api.v1.map_routes import (
    set_search_space_roi,
    get_active_roi,
    clear_active_roi
)

def run_direct_tests():
    print("[*] Starting Phase 02 Schema & Route Validation...")

    # 1. Valid Rectangle Schema
    req_rect = SearchSpaceROIRequest(
        roi_type="RECTANGLE",
        bounds=BoundingBox(lat_min=33.30, lat_max=33.325, lon_min=44.35, lon_max=44.38),
        vertices=[],
        area_km2=7.68
    )
    assert len(req_rect.vertices) == 4, "Rectangle should automatically build 4 vertices"
    print("  [✓] Rectangle auto-vertex generation passed.")

    # 2. Invoke route handler
    res = set_search_space_roi(req_rect)
    assert res.status == "VALIDATED_AND_ACTIVE"
    assert res.vertices_count == 4
    assert res.estimated_50m_cells == int(7.68 / 0.0025)
    print("  [✓] set_search_space_roi() executed & cells estimated correctly.")

    # 3. Get active ROI
    active = get_active_roi()
    assert active["area_km2"] == 7.68
    print("  [✓] get_active_roi() verified.")

    # 4. Valid Polygon Schema
    req_poly = SearchSpaceROIRequest(
        roi_type="POLYGON",
        bounds=BoundingBox(lat_min=33.30, lat_max=33.35, lon_min=44.35, lon_max=44.40),
        vertices=[
            GeoPoint(lat=33.30, lon=44.35),
            GeoPoint(lat=33.35, lon=44.36),
            GeoPoint(lat=33.33, lon=44.40)
        ],
        area_km2=12.4
    )
    res_poly = set_search_space_roi(req_poly)
    assert res_poly.vertices_count == 3
    print("  [✓] Polygon ROI registered successfully.")

    # 5. Invalid Polygon (< 3 vertices)
    try:
        SearchSpaceROIRequest(
            roi_type="POLYGON",
            bounds=BoundingBox(lat_min=33.30, lat_max=33.35, lon_min=44.35, lon_max=44.40),
            vertices=[
                GeoPoint(lat=33.30, lon=44.35),
                GeoPoint(lat=33.35, lon=44.40)
            ],
            area_km2=5.0
        )
        assert False, "Should have failed with ValidationError"
    except ValidationError:
        print("  [✓] Rejection of polygon with < 3 vertices passed.")

    # 6. Invalid Bounds (inverted min/max)
    try:
        BoundingBox(lat_min=33.50, lat_max=33.20, lon_min=44.35, lon_max=44.40)
        assert False, "Should have failed with ValidationError"
    except ValidationError:
        print("  [✓] Rejection of inverted geodetic bounds passed.")

    # 7. Invalid Area (> 500 km2)
    try:
        SearchSpaceROIRequest(
            roi_type="RECTANGLE",
            bounds=BoundingBox(lat_min=30.0, lat_max=40.0, lon_min=40.0, lon_max=50.0),
            vertices=[],
            area_km2=850.0
        )
        assert False, "Should have failed with ValidationError"
    except ValidationError:
        print("  [✓] Rejection of excessive mission area passed.")

    # 8. Clear active ROI
    del_res = clear_active_roi()
    assert del_res["status"] == "CLEARED"
    print("  [✓] clear_active_roi() executed.")

    print("\n[SUCCESS] All Phase 02 Schema & Business Logic Tests Passed Successfully!")

if __name__ == "__main__":
    run_direct_tests()
