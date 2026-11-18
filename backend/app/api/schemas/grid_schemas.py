from typing import List, Optional, Dict, Any, Literal
from pydantic import BaseModel, Field
from app.api.schemas.geo_schemas import BoundingBox, GeoPoint

class GridGenerationRequest(BaseModel):
    """Payload to trigger Geodesic Grid Discretization."""
    roi_type: Literal["RECTANGLE", "POLYGON"] = "RECTANGLE"
    bounds: BoundingBox
    vertices: List[GeoPoint] = Field(default_factory=list)
    altitude_m: float = Field(default=60.0, ge=10.0, le=500.0, description="UAV Operating Altitude (meters)")
    sensor_fov_deg: float = Field(default=70.0, ge=20.0, le=140.0, description="Camera/LiDAR Field of View (degrees)")
    manual_cell_size_m: Optional[float] = Field(default=None, ge=5.0, le=500.0, description="Optional manual cell dimension override")
    overlap_ratio: float = Field(default=0.10, ge=0.0, le=0.80, description="Strip overlap ratio (0.0 to 0.8)")

class GridMetadataResponse(BaseModel):
    rows: int
    cols: int
    total_cells: int
    active_cells: int
    masked_cells: int
    cell_size_m: float
    delta_lat_deg: float
    delta_lon_deg: float
    mean_latitude: float

class GridGenerationResponse(BaseModel):
    status: str = "GRID_GENERATED_AND_INDEXED"
    metadata: GridMetadataResponse
    geojson: Dict[str, Any]
    message: str = "Search space successfully discretized and indexed in O(1) memory table."


class HybridGridGenerationRequest(BaseModel):
    """Payload to trigger Context-Aware Hybrid Graph-Grid Discretization."""
    roi_type: Literal["RECTANGLE", "POLYGON"] = "POLYGON"
    bounds: BoundingBox
    vertices: List[GeoPoint] = Field(default_factory=list)
    cluster_threshold_m: float = Field(default=25.0, ge=10.0, le=150.0, description="Spatial node clustering radius in meters")
    altitude_m: float = Field(default=60.0, ge=10.0, le=500.0, description="UAV operating altitude for sensor coverage")
    sensor_fov_deg: float = Field(default=70.0, ge=20.0, le=140.0)
    manual_cell_size_m: Optional[float] = Field(default=None, ge=15.0, le=500.0)
    buffer_radius_ratio: float = Field(default=0.65, ge=0.3, le=1.5, description="Ratio of cell size for road spatial exclusion buffer")
    force_fallback: bool = Field(default=False, description="Forces open-terrain fallback without contacting Overpass API")


class HybridSpaceStatistics(BaseModel):
    total_targets: int
    road_intersection_count: int
    open_terrain_cells_count: int
    spatial_excluded_cells_count: int
    road_edges_count: int
    buffer_radius_m: float
    zero_overlap_verified: bool = True
    source: str


class HybridGridGenerationResponse(BaseModel):
    status: str = "HYBRID_SPACE_GENERATED"
    mission_id: Optional[str] = None
    home_base: Optional[Dict[str, float]] = None
    statistics: HybridSpaceStatistics
    targets: List[Dict[str, Any]]
    geojson: Dict[str, Any]
    message: str = "Context-aware hybrid graph-grid search space generated with zero-overlap spatial exclusion."

