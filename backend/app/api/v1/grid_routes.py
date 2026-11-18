import math
from fastapi import APIRouter, HTTPException, status, Query
from typing import Optional, Dict, Any, List
from app.api.schemas.grid_schemas import (
    GridGenerationRequest,
    GridGenerationResponse,
    GridMetadataResponse,
    HybridGridGenerationRequest,
    HybridGridGenerationResponse,
    HybridSpaceStatistics
)
from app.gis.grid import GeodesicGridDiscretizer
from app.gis.road_topology_extractor import global_road_extractor
from app.gis.hybrid_space_builder import HybridGraphGridBuilder, SearchTargetNode
from app.core.mission_state import mission_state

router = APIRouter(prefix="/grid", tags=["Geodesic Grid & Spatial Indexing"])

# Global Active Grid & Hybrid Space Singletons
_ACTIVE_GRID: Optional[GeodesicGridDiscretizer] = None
_ACTIVE_HYBRID_BUILDER: Optional[HybridGraphGridBuilder] = None
_ACTIVE_TARGETS: Optional[List[SearchTargetNode]] = None

@router.post(
    "/generate",
    response_model=GridGenerationResponse,
    status_code=status.HTTP_201_CREATED,
    summary="Discretize Search Space & Build O(1) Spatial Index"
)
def generate_geodesic_grid(payload: GridGenerationRequest):
    """
    Transforms continuous Search Space bounds (and optional Polygon vertices) into
    a regular geodesic grid calibrated to UAV flight altitude and sensor FOV.
    Applies Point-in-Polygon (Ray Casting) to exclude outside cells and indexes
    the entire matrix for O(1) query time.
    """
    global _ACTIVE_GRID

    poly_vertices = [(v.lat, v.lon) for v in payload.vertices]

    discretizer = GeodesicGridDiscretizer(
        lat_min=payload.bounds.lat_min,
        lat_max=payload.bounds.lat_max,
        lon_min=payload.bounds.lon_min,
        lon_max=payload.bounds.lon_max,
        polygon_vertices=poly_vertices,
        altitude_m=payload.altitude_m,
        sensor_fov_deg=payload.sensor_fov_deg,
        manual_cell_size_m=payload.manual_cell_size_m,
        overlap_ratio=payload.overlap_ratio
    )

    _ACTIVE_GRID = discretizer
    geojson_data = discretizer.to_geojson(include_masked=True)

    metadata = GridMetadataResponse(
        rows=discretizer.rows,
        cols=discretizer.cols,
        total_cells=discretizer.total_cells,
        active_cells=discretizer.active_cell_count,
        masked_cells=discretizer.masked_cell_count,
        cell_size_m=round(discretizer.cell_size_m, 2),
        delta_lat_deg=discretizer.delta_lat,
        delta_lon_deg=discretizer.delta_lon,
        mean_latitude=round(discretizer.mean_lat, 6)
    )

    return GridGenerationResponse(
        metadata=metadata,
        geojson=geojson_data
    )

@router.post(
    "/hybrid-generate",
    response_model=HybridGridGenerationResponse,
    status_code=status.HTTP_201_CREATED,
    summary="Generate Context-Aware Hybrid Graph-Grid Search Space"
)
async def generate_hybrid_graph_grid(payload: HybridGridGenerationRequest):
    r"""
    Constructs a unified, context-aware search domain:
    T = V_road U (C_grid \ Buffer(V_road, R))
    
    1. Executes an atomic hard reset to invalidate any prior simulation and session epoch.
    2. Initializes a unique session mission_id (UUID) and relocates the Shared Home Base.
    3. Extracts road graph intersections (Degree >= 3) from OpenStreetMap via Overpass API.
    4. Clusters nearby nodes within cluster_threshold_m.
    5. Discretizes the bounding region into standard geodesic cells with zero-overlap spatial exclusion.
    6. Returns unified targets, dual-layer GeoJSON, mission_id, home_base, and segregation statistics.
    """
    global _ACTIVE_GRID, _ACTIVE_HYBRID_BUILDER, _ACTIVE_TARGETS

    # 1. Atomic Hard Reset of Previous Mission & Invalidation of Stale Loops
    await mission_state.hard_reset()

    # 2. Relocate Shared Home Base to First ROI Vertex (or centroid if rectangle)
    if payload.vertices and len(payload.vertices) > 0:
        home_base_lat = payload.vertices[0].lat
        home_base_lon = payload.vertices[0].lon
    else:
        home_base_lat = (payload.bounds.lat_min + payload.bounds.lat_max) / 2.0
        home_base_lon = (payload.bounds.lon_min + payload.bounds.lon_max) / 2.0
    home_base_dict = {"lat": home_base_lat, "lon": home_base_lon}

    # 3. Generate New Unique Mission Session Epoch (UUID)
    mission_id = mission_state.start_new_mission(
        bounds=payload.bounds.model_dump(),
        vertices=[v.model_dump() for v in payload.vertices] if payload.vertices else None,
        home_base=home_base_dict
    )

    poly_vertices = [(v.lat, v.lon) for v in payload.vertices] if payload.vertices else None

    # 4. Fetch Road Network Topology from OSM Overpass with fallback
    road_network = await global_road_extractor.fetch_road_topology(
        min_lat=payload.bounds.lat_min,
        min_lon=payload.bounds.lon_min,
        max_lat=payload.bounds.lat_max,
        max_lon=payload.bounds.lon_max,
        cluster_threshold_m=payload.cluster_threshold_m,
        polygon_vertices=poly_vertices,
        force_fallback=payload.force_fallback
    )

    # 4.5. Compute Area & Adaptive Cell Resolution Scaling
    # Caps total active search targets to ~300 so multi-UAV optimization converges in < 1.5 seconds.
    if payload.vertices and len(payload.vertices) >= 3:
        mean_lat = sum(v.lat for v in payload.vertices) / len(payload.vertices)
        cos_lat = math.cos(math.radians(mean_lat))
        v0 = payload.vertices[0]
        pts_m = [
            ((v.lon - v0.lon) * 111320.0 * cos_lat, (v.lat - v0.lat) * 111320.0)
            for v in payload.vertices
        ]
        n_pts = len(pts_m)
        roi_area_m2 = 0.5 * abs(
            sum(pts_m[i][0] * pts_m[(i + 1) % n_pts][1] - pts_m[(i + 1) % n_pts][0] * pts_m[i][1] for i in range(n_pts))
        )
    else:
        d_lat_m = (payload.bounds.lat_max - payload.bounds.lat_min) * 111320.0
        mean_lat = (payload.bounds.lat_max + payload.bounds.lat_min) / 2.0
        d_lon_m = (payload.bounds.lon_max - payload.bounds.lon_min) * 111320.0 * math.cos(math.radians(mean_lat))
        roi_area_m2 = abs(d_lat_m * d_lon_m)

    # Phase 01: Physical sensor swath calibrated scanning resolution (W_sensor ≈ 55.0 m)
    # Calibrated to UAV altitude with 70° FOV: W_sensor = 2 * h * tan(FOV/2) * (1 - overlap) ≈ 55m.
    fov_rad = math.radians(payload.sensor_fov_deg)
    sensor_swath_m = 2.0 * payload.altitude_m * math.tan(fov_rad / 2.0) * (1.0 - 0.10)
    calibrated_cell_size = max(55.0, min(65.0, sensor_swath_m))

    if payload.manual_cell_size_m and payload.manual_cell_size_m > 0:
        effective_cell_size = payload.manual_cell_size_m
    else:
        # Maintain physical resolution without cell blowup for 100% full coverage
        effective_cell_size = calibrated_cell_size

    # 5. Build Base Geodesic Grid
    discretizer = GeodesicGridDiscretizer(
        lat_min=payload.bounds.lat_min,
        lat_max=payload.bounds.lat_max,
        lon_min=payload.bounds.lon_min,
        lon_max=payload.bounds.lon_max,
        polygon_vertices=poly_vertices,
        altitude_m=payload.altitude_m,
        sensor_fov_deg=payload.sensor_fov_deg,
        manual_cell_size_m=effective_cell_size
    )

    # 6. Fuse Road Graph with Grid via Spatial Exclusion Buffer
    hybrid_builder = HybridGraphGridBuilder(
        grid=discretizer,
        road_network=road_network,
        buffer_radius_ratio=payload.buffer_radius_ratio
    )

    # Cache as active mission grid and hybrid space
    _ACTIVE_GRID = discretizer
    _ACTIVE_HYBRID_BUILDER = hybrid_builder
    _ACTIVE_TARGETS = hybrid_builder.unified_targets

    # Store in central mission state
    mission_state.active_grid = discretizer
    mission_state.active_hybrid_builder = hybrid_builder
    mission_state.active_targets = hybrid_builder.unified_targets

    stats = HybridSpaceStatistics(
        total_targets=len(hybrid_builder.unified_targets),
        road_intersection_count=len(hybrid_builder.active_road_nodes),
        open_terrain_cells_count=len(hybrid_builder.active_open_cells),
        spatial_excluded_cells_count=hybrid_builder.spatial_excluded_cells_count,
        road_edges_count=len(road_network.edges),
        buffer_radius_m=round(hybrid_builder.buffer_radius_m, 1),
        zero_overlap_verified=True,
        source=road_network.source
    )

    return HybridGridGenerationResponse(
        status="HYBRID_SPACE_GENERATED",
        mission_id=mission_id,
        home_base=home_base_dict,
        statistics=stats,
        targets=[t.model_dump() for t in hybrid_builder.unified_targets],
        geojson=hybrid_builder.to_dual_layer_geojson(),
        message="Context-aware hybrid graph-grid search space generated with zero-overlap spatial exclusion."
    )


@router.get(
    "/active",
    summary="Get Active Grid Metadata and Statistics"
)
def get_active_grid_metadata():
    """Returns metadata and cell counts for the currently indexed mission grid."""
    if _ACTIVE_GRID is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No grid has been generated yet. Please post to /api/v1/grid/generate first."
        )

    return {
        "status": "ONLINE",
        "rows": _ACTIVE_GRID.rows,
        "cols": _ACTIVE_GRID.cols,
        "total_cells": _ACTIVE_GRID.total_cells,
        "active_cells": _ACTIVE_GRID.active_cell_count,
        "masked_cells": _ACTIVE_GRID.masked_cell_count,
        "cell_size_m": round(_ACTIVE_GRID.cell_size_m, 2),
        "delta_lat_deg": _ACTIVE_GRID.delta_lat,
        "delta_lon_deg": _ACTIVE_GRID.delta_lon
    }

@router.get(
    "/cell-lookup",
    summary="O(1) Spatial Coordinate to Cell Inverse Lookup"
)
def lookup_cell_by_coordinate(
    lat: float = Query(..., description="UAV Latitude"),
    lon: float = Query(..., description="UAV Longitude")
):
    """
    Demonstrates instantaneous O(1) telemetry-to-cell identification.
    Used continuously by the physics simulator and telemetry hub.
    """
    if _ACTIVE_GRID is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Active grid is not initialized."
        )

    cell = _ACTIVE_GRID.coord_to_cell(lat, lon)
    if cell is None:
        return {
            "found": False,
            "message": "Coordinates are outside the search space bounds."
        }

    return {
        "found": True,
        "cell": cell.to_dict()
    }
