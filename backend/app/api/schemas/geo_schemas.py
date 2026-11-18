from typing import List, Literal, Optional
from pydantic import BaseModel, Field, field_validator, model_validator
import uuid
from datetime import datetime

class GeoPoint(BaseModel):
    """Geodetic Coordinate Point in WGS84 (EPSG:4326)."""
    lat: float = Field(..., ge=-90.0, le=90.0, description="Latitude in decimal degrees (-90 to 90)")
    lon: float = Field(..., ge=-180.0, le=180.0, description="Longitude in decimal degrees (-180 to 180)")

class BoundingBox(BaseModel):
    """Enclosing Geodetic Bounding Box of the Search Space."""
    lat_min: float = Field(..., ge=-90.0, le=90.0)
    lat_max: float = Field(..., ge=-90.0, le=90.0)
    lon_min: float = Field(..., ge=-180.0, le=180.0)
    lon_max: float = Field(..., ge=-180.0, le=180.0)

    @model_validator(mode='after')
    def validate_bounds_order(self):
        if self.lat_min >= self.lat_max:
            raise ValueError("lat_min must be strictly less than lat_max")
        if self.lon_min >= self.lon_max:
            raise ValueError("lon_min must be strictly less than lon_max")
        return self

class SearchSpaceROIRequest(BaseModel):
    """Payload sent by the Frontend when the user completes drawing an ROI."""
    roi_type: Literal["RECTANGLE", "POLYGON"] = Field(..., description="Type of geometric bounding drawn")
    bounds: BoundingBox = Field(..., description="Geodetic bounding box enclosing the search space")
    vertices: List[GeoPoint] = Field(default_factory=list, description="Array of polygon vertices in sequential order")
    area_km2: float = Field(..., gt=0.0001, le=500.0, description="Calculated area in square kilometers (Max 500 km2 for UAV feasibility)")

    @model_validator(mode='after')
    def validate_geometry(self):
        if self.roi_type == "POLYGON":
            if len(self.vertices) < 3:
                raise ValueError("Polygon ROI must contain at least 3 vertices to form a closed 2D area.")
        elif self.roi_type == "RECTANGLE":
            # For rectangle, if vertices are not provided, reconstruct 4 vertices from bounding box
            if not self.vertices:
                self.vertices = [
                    GeoPoint(lat=self.bounds.lat_min, lon=self.bounds.lon_min),
                    GeoPoint(lat=self.bounds.lat_max, lon=self.bounds.lon_min),
                    GeoPoint(lat=self.bounds.lat_max, lon=self.bounds.lon_max),
                    GeoPoint(lat=self.bounds.lat_min, lon=self.bounds.lon_max),
                ]
        return self

class SearchSpaceROIResponse(BaseModel):
    """Response returned upon successful validation and caching of Search Space."""
    status: str = "VALIDATED_AND_ACTIVE"
    roi_id: str = Field(default_factory=lambda: f"roi-{uuid.uuid4().hex[:8]}")
    roi_type: str
    bounds: BoundingBox
    vertices_count: int
    area_km2: float
    estimated_50m_cells: int
    created_at: str = Field(default_factory=lambda: datetime.utcnow().isoformat())
    message: str = "Search space ROI registered successfully. Ready for Geodesic Discretization."
