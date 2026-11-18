"""Re-exports for Phase 04 physics modules."""
from physics.flight_physics import (
    haversine_distance,
    LiDARTerrainProcessor,
    UAVPhysicsEngine
)
from physics.synthetic_terrain import SyntheticTerrainGenerator

__all__ = [
    "haversine_distance",
    "LiDARTerrainProcessor",
    "UAVPhysicsEngine",
    "SyntheticTerrainGenerator"
]
