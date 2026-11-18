from fastapi import APIRouter, HTTPException, status
from typing import Optional
from app.api.schemas.geo_schemas import SearchSpaceROIRequest, SearchSpaceROIResponse

router = APIRouter(prefix="/map", tags=["GIS Map & Search Space"])

# In-memory storage for the active mission ROI
_ACTIVE_MISSION_ROI: Optional[dict] = None

@router.post(
    "/roi",
    response_model=SearchSpaceROIResponse,
    status_code=status.HTTP_201_CREATED,
    summary="Validate and Register Search Space ROI"
)
def set_search_space_roi(payload: SearchSpaceROIRequest):
    """
    Receives bounding box and vertices drawn via mouse in the Frontend GIS view.
    Validates spherical boundaries, prevents zero or unrealistic search spaces,
    and caches the active mission domain for subsequent grid discretization (Phase 03).
    """
    global _ACTIVE_MISSION_ROI

    # Estimate discrete 50m x 50m cells (1 cell = 2,500 m^2 = 0.0025 km^2)
    estimated_cells = int(payload.area_km2 / 0.0025)
    if estimated_cells < 1:
        estimated_cells = 1

    response = SearchSpaceROIResponse(
        roi_type=payload.roi_type,
        bounds=payload.bounds,
        vertices_count=len(payload.vertices),
        area_km2=round(payload.area_km2, 4),
        estimated_50m_cells=estimated_cells
    )

    # Cache active ROI in server memory
    _ACTIVE_MISSION_ROI = {
        "response": response.model_dump(),
        "request": payload.model_dump()
    }

    # Automatically discretize and activate grid for immediate optimization
    try:
        from app.gis.grid import GeodesicGridDiscretizer
        import app.api.v1.grid_routes as grid_routes
        poly_verts = [(v.lat, v.lon) for v in payload.vertices] if payload.vertices else None
        
        # Adaptive cell size based on area to ensure rapid solver convergence (< 1 sec)
        if payload.area_km2 <= 1.0:
            cell_size = 90.0
        elif payload.area_km2 <= 3.0:
            cell_size = 140.0
        elif payload.area_km2 <= 8.0:
            cell_size = 200.0
        else:
            cell_size = 280.0

        discretizer = GeodesicGridDiscretizer(
            lat_min=payload.bounds.lat_min,
            lat_max=payload.bounds.lat_max,
            lon_min=payload.bounds.lon_min,
            lon_max=payload.bounds.lon_max,
            polygon_vertices=poly_verts,
            manual_cell_size_m=cell_size
        )
        # Only set active grid fallback if no hybrid grid exists
        if getattr(grid_routes, "_ACTIVE_GRID", None) is None:
            grid_routes._ACTIVE_GRID = discretizer

        # Reposition fleet base station to the centroid of the new search space
        from app.fleet.drone_registry import fleet_registry
        from app.fleet.telemetry_models import DroneNodeConfig
        current_configs = fleet_registry.get_ordered_configs()
        num_drones = len(current_configs) if current_configs else 3
        mean_lat = discretizer.mean_lat
        mean_lon = (payload.bounds.lon_min + payload.bounds.lon_max) / 2.0
        fleet_registry.clear_all()
        for i in range(1, num_drones + 1):
            fleet_registry.add_drone(DroneNodeConfig(
                drone_id=f"UAV-{i:02d}",
                endpoint_uri=f"sim://uav-{i:02d}",
                cruise_speed_mps=14.0 + (i * 0.4),
                battery_capacity_wh=120.0,
                initial_lat=mean_lat,
                initial_lon=mean_lon
            ))
    except Exception as e:
        print(f"[!] Warning: Grid auto-discretization error: {e}")

    return response

@router.get(
    "/roi",
    summary="Get Current Active Search Space ROI"
)
def get_active_roi():
    """Returns the currently active search space ROI or 404 if not yet defined."""
    if _ACTIVE_MISSION_ROI is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No search space ROI has been drawn or registered yet."
        )
    return _ACTIVE_MISSION_ROI["response"]

@router.delete(
    "/roi",
    summary="Clear Active Search Space ROI"
)
def clear_active_roi():
    """Resets the active search space to allow re-drawing."""
    global _ACTIVE_MISSION_ROI
    _ACTIVE_MISSION_ROI = None
    try:
        import app.api.v1.grid_routes as grid_routes
        grid_routes._ACTIVE_GRID = None
        grid_routes._ACTIVE_HYBRID_BUILDER = None
        grid_routes._ACTIVE_TARGETS = None
    except Exception:
        pass
    return {"status": "CLEARED", "message": "Search space ROI cleared successfully."}
